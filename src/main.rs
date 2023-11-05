#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(clippy::upper_case_acronyms)]

extern crate alloc;
use core::{alloc::GlobalAlloc, ops::ControlFlow, str};
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Config, Stack, StackResources,
};
use embassy_time::{Duration, Timer};
use embedded_graphics::{pixelcolor::Gray8, prelude::Dimensions};
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiMode, WifiState};
use esp_wifi::{initialize, EspWifiInitFor};

#[cfg(feature = "esp32c3")]
pub use esp32c3_hal as hal;
#[cfg(feature = "esp32c6")]
pub use esp32c6_hal as hal;

use hal::{
    clock::ClockControl, embassy, gpio::*, peripherals::Peripherals, prelude::*, timer::TimerGroup,
    Rtc, IO,
};
use hal::{systimer::SystemTimer, Rng};

use embedded_graphics::draw_target::DrawTarget;
use embedded_io_async::Read;
use incremental_png::{
    dechunker::Dechunker,
    inflater::{self, Inflater},
    stream_decoder::{ImageHeader, StreamDecoder},
};
use reqwless::client::HttpClient;
use reqwless::request::Method;
use static_cell::make_static;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// Ideally we'd like to get rid of the allocator, but some dependency requires it and it's hard to
// find out which one.
// See
// <https://users.rust-lang.org/t/rust-no-std-find-why-global-memory-allocator-is-required/77679>
#[global_allocator]
static ALLOCATOR: FakeAllocator = FakeAllocator;

struct FakeAllocator;

unsafe impl GlobalAlloc for FakeAllocator {
    unsafe fn alloc(&self, _: core::alloc::Layout) -> *mut u8 {
        panic!("who needs the allocator?");
    }
    unsafe fn dealloc(&self, _: *mut u8, _: core::alloc::Layout) {
        panic!("who needs the allocator?");
    }
    unsafe fn alloc_zeroed(&self, _: core::alloc::Layout) -> *mut u8 {
        panic!("who needs the allocator?");
    }
    unsafe fn realloc(&self, _: *mut u8, _: core::alloc::Layout, _: usize) -> *mut u8 {
        panic!("who needs the allocator?");
    }
}

#[cfg(feature = "display-st7735")]
mod display {
    use super::*;
    use embedded_graphics::pixelcolor::{raw::RawU16, Rgb565, RgbColor};
    use embedded_graphics::prelude::RawData;
    use hal::peripherals::SPI2;
    use hal::spi::FullDuplexMode;
    use hal::Spi;
    use st7735_lcd::{self, ST7735};

    pub type SPI = Spi<'static, SPI2, FullDuplexMode>;
    pub type DISPLAY<'a> = ST7735<SPI, GpioPin<Output<PushPull>, 6>, GpioPin<Output<PushPull>, 7>>;

    pub type Color = Rgb565;
    pub const BACKGROUND: Color = Rgb565::BLACK;
    pub const TEXT: Color = Rgb565::RED;

    pub fn flush(_display: &mut DISPLAY) -> Result<(), ()> {
        // no-op
        Ok(())
    }

    pub fn set_pixel(display: &mut DISPLAY, x: u32, y: u32, color: Color) -> Result<(), ()> {
        display.set_pixel(x as u16, y as u16, RawU16::from(color).into_inner())
    }
}

#[cfg(feature = "display-st7789")]
mod display {
    use super::*;
    use display_interface::DisplayError;
    use display_interface_spi::SPIInterfaceNoCS;
    use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
    use hal::peripherals::SPI2;
    use hal::spi::FullDuplexMode;
    use hal::Spi;
    use mipidsi::models::ST7789;

    pub type SPI = Spi<'static, SPI2, FullDuplexMode>;
    pub type DISPLAY<'a> = mipidsi::Display<
        SPIInterfaceNoCS<
            Spi<'a, esp32c3_hal::peripherals::SPI2, FullDuplexMode>,
            GpioPin<Output<esp32c3_hal::gpio::PushPull>, 6>,
        >,
        ST7789,
        GpioPin<Output<esp32c3_hal::gpio::PushPull>, 7>,
    >;

    pub type Color = Rgb565;
    pub const BACKGROUND: Color = Rgb565::BLACK;
    pub const TEXT: Color = Rgb565::RED;

    pub fn flush(_display: &mut DISPLAY) -> Result<(), ()> {
        // no-op
        Ok(())
    }

    pub fn set_pixel(
        display: &mut DISPLAY,
        x: u32,
        y: u32,
        color: Color,
    ) -> Result<(), DisplayError> {
        display.set_pixel(x as u16, y as u16, color)
    }

    pub fn write_row(
        display: &mut DISPLAY,
        x: u32,
        y: u32,
        n: u32,
        colors: impl Iterator<Item = Color>,
    ) -> Result<(), DisplayError> {
        display.set_pixels(
            x as u16,
            y as u16,
            x as u16 + n as u16 - 1,
            y as u16,
            colors,
        )
    }
}

#[cfg(feature = "display-ssd1306")]
mod display {
    use super::*;
    use embedded_graphics::pixelcolor::BinaryColor;
    use hal::i2c::I2C;
    use hal::peripherals::I2C0;
    use ssd1306::prelude::I2CInterface;
    use ssd1306::{mode::BufferedGraphicsMode, size::DisplaySize128x64, Ssd1306};

    pub type SIZE = DisplaySize128x64;
    pub const SIZE: SIZE = DisplaySize128x64;
    pub type DISPLAY<'a> = Ssd1306<I2CInterface<I2C<'a, I2C0>>, SIZE, BufferedGraphicsMode<SIZE>>;

    pub type Color = BinaryColor;
    pub const BACKGROUND: Color = BinaryColor::Off;
    pub const TEXT: Color = BinaryColor::On;

    pub fn flush(display: &mut DISPLAY) -> Result<(), display_interface::DisplayError> {
        display.flush()
    }

    pub fn set_pixel(display: &mut DISPLAY, x: u32, y: u32, color: Color) -> Result<(), ()> {
        // Note: we invert pixels, because xkcd looks better that way on this display
        display.set_pixel(x, y, color != BinaryColor::On);
        Ok(())
    }
}

#[cfg(feature = "display-ili9488")]
mod display {
    use super::*;
    use display_interface::DisplayError;
    use display_interface_spi::SPIInterfaceNoCS;
    use embedded_graphics::pixelcolor::Rgb666;
    use embedded_graphics::prelude::RgbColor;
    use hal::peripherals::SPI2;
    use hal::spi::FullDuplexMode;
    use hal::Spi;
    use mipidsi::*;

    pub type SPI = Spi<'static, SPI2, FullDuplexMode>;
    pub type DISPLAY<'a> = Display<
        SPIInterfaceNoCS<SPI, GpioPin<Output<PushPull>, 6>>,
        models::ILI9486Rgb666,
        GpioPin<Output<PushPull>, 7>,
    >;

    pub type Color = Rgb666;
    pub const BACKGROUND: Color = Rgb666::BLACK;
    pub const TEXT: Color = Rgb666::RED;

    pub fn flush(_display: &mut DISPLAY) -> Result<(), ()> {
        // no-op
        Ok(())
    }

    pub fn set_pixel(
        display: &mut DISPLAY,
        x: u32,
        y: u32,
        color: Color,
    ) -> Result<(), DisplayError> {
        display.set_pixel(x as u16, y as u16, color)
    }
}

use display::DISPLAY;

#[embassy_executor::main(entry = "hal::entry")]
async fn main(spawner: embassy_executor::Spawner) {
    let peripherals = Peripherals::take();
    #[cfg(feature = "esp32c3")]
    let mut system = peripherals.SYSTEM.split();
    #[cfg(feature = "esp32c6")]
    let mut system = peripherals.PCR.split();

    let clocks = ClockControl::max(system.clock_control).freeze();

    #[cfg(feature = "esp32c3")]
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    #[cfg(feature = "esp32c6")]
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    esp_println::logger::init_logger(log::LevelFilter::Info);

    let mut rng = Rng::new(peripherals.RNG);
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        rng,
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let (wifi, ..) = peripherals.RADIO.split();
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiMode::Sta).unwrap();

    embassy::init(&clocks, timer_group0.timer0);

    let dhcp4_config = embassy_net::DhcpConfig::default();
    let config = Config::dhcpv4(dhcp4_config);

    let seed = rng.random();

    let stack = &*make_static!(Stack::new(
        wifi_interface,
        config,
        make_static!(StackResources::<3>::new()),
        seed.into()
    ));

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let input = io.pins.gpio9.into_pull_up_input();

    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    use display::*;

    #[cfg(any(feature = "display-st7735", feature = "display-st7789"))]
    let mut display: DISPLAY = {
        use hal::{Delay, Spi};

        let mut delay = Delay::new(&clocks);

        let a0 = io.pins.gpio6.into_push_pull_output();
        let rst = io.pins.gpio7.into_push_pull_output();

        let sck = io.pins.gpio1;
        let sda = io.pins.gpio2;
        let cs = io.pins.gpio8;

        let spi: SPI = Spi::new(
            peripherals.SPI2,
            sck,
            sda,
            io.pins.gpio0.into_push_pull_output(), // no MISO, dummy pin
            cs,
            60u32.MHz(),
            hal::spi::SpiMode::Mode0,
            &mut system.peripheral_clock_control,
            &clocks,
        );

        #[cfg(feature = "display-st7735")]
        {
            let mut display = DISPLAY::new(spi, a0, rst, true, false, 160, 128);
            display.init(&mut delay).unwrap();
            display
                .set_orientation(&st7735_lcd::Orientation::Landscape)
                .unwrap();
            display.set_offset(0, 0);
            display
        }

        #[cfg(feature = "display-st7789")]
        {
            use display_interface_spi::SPIInterfaceNoCS;
            let interface = SPIInterfaceNoCS::new(spi, a0);
            let display = mipidsi::Builder::st7789(interface)
                .with_orientation(mipidsi::Orientation::LandscapeInverted(true))
                .init(&mut delay, Some(rst))
                .unwrap();
            display
        }
    };

    #[cfg(feature = "display-ssd1306")]
    let mut display: DISPLAY = {
        use hal::i2c::I2C;
        use ssd1306::prelude::*;
        use ssd1306::rotation::DisplayRotation;
        use ssd1306::*;

        let sda = io.pins.gpio1;
        let scl = io.pins.gpio2;

        let i2c = I2C::new(
            peripherals.I2C0,
            sda,
            scl,
            400u32.kHz(),
            &mut system.peripheral_clock_control,
            &clocks,
        );

        let interface = I2CDisplayInterface::new(i2c);
        let mut display =
            Ssd1306::new(interface, SIZE, DisplayRotation::Rotate0).into_buffered_graphics_mode();

        display.init().unwrap();
        display
    };

    #[cfg(feature = "display-ili9488")]
    let mut display: DISPLAY = {
        use display_interface_spi::SPIInterfaceNoCS;
        use hal::{spi::SpiMode, Delay, Spi};
        use mipidsi::*;

        // Define the Data/Command select pin as a digital output
        let dc = io.pins.gpio6.into_push_pull_output();
        // Define the reset pin as digital outputs and make it high
        let mut rst = io.pins.gpio7.into_push_pull_output();
        rst.set_high().unwrap();

        let mut backlight = io.pins.gpio3.into_push_pull_output();
        backlight.set_high().unwrap();
        // Define the SPI pins and create the SPI interface
        let sck = io.pins.gpio4;
        let miso = io.pins.gpio2;
        let mosi = io.pins.gpio5;
        let cs = io.pins.gpio8;
        let spi = Spi::new(
            peripherals.SPI2,
            sck,
            mosi,
            miso,
            cs,
            60_u32.MHz(),
            SpiMode::Mode2,
            &mut system.peripheral_clock_control,
            &clocks,
        );

        // Define the display interface with no chip select
        let di = SPIInterfaceNoCS::new(spi, dc);

        let mut delay = Delay::new(&clocks);
        let display = Builder::ili9486_rgb666(di)
            .with_orientation(Orientation::PortraitInverted(true))
            .init(&mut delay, Some(rst))
            .unwrap();

        display
    };

    display.clear(display::BACKGROUND).unwrap();
    display::flush(&mut display).unwrap();

    spawner.spawn(connection_wifi(controller)).ok();
    spawner.spawn(net_task(stack)).ok();
    spawner
        .spawn(task(input, stack, seed.into(), display, rtc))
        .ok();
}

#[embassy_executor::task]
async fn task(
    mut input: Gpio9<Input<PullUp>>,
    stack: &'static Stack<WifiDevice<'static>>,
    _seed: u64,
    mut display: DISPLAY<'static>,
    rtc: Rtc<'static>,
) {
    let mut rx_buffer = [0; 2048];
    let client_state = TcpClientState::<1, 2048, 2048>::new();
    let tcp_client = TcpClient::new(stack, &client_state);
    let dns = DnsSocket::new(stack);

    const IMAGE_URLS: &[&str] = &[
        "http://imgs.xkcd.com/comics/book_burning.png",
        "http://imgs.xkcd.com/comics/the_universal_label.png",
        "http://imgs.xkcd.com/comics/journal_4.png",
        "http://imgs.xkcd.com/comics/daylight_saving_choice.png",
        // biggest grayscale image - 549K
        "http://imgs.xkcd.com/comics/the_pace_of_modern_life.png",
        // non-text
        "http://imgs.xkcd.com/comics/to_be_wanted.png",
        // Doesn't work, we only handle grayscale images for now
        "http://imgs.xkcd.com/comics/dendrochronology.png",
        // Crashes with "buffer error" from incremental-png :(
        // "http://imgs.xkcd.com/comics/depth.png",

        // We don't handle this color type
        // "http://imgs.xkcd.com/comics/breaker_box.png",
    ];

    let display_width = display.bounding_box().size.width;
    let display_height = display.bounding_box().size.height;
    let mut image_index = 0;
    let mut image_offset_x: u32 = 0;
    let mut image_offset_y: u32 = 0;

    loop {
        stack.wait_config_up().await;
        loop {
            if let Some(config) = stack.config_v4() {
                println!("Got IP: {}", config.address);
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        display.clear(display::TEXT).unwrap();
        display::flush(&mut display).unwrap();

        // FIXME: HTTPS doesn't work on imgs.xkcd.com: Tls(HandshakeAborted(Fatal, ProtocolVersion))
        // let mut tls_read_buffer = [0; 8 * 1024];
        // let mut tls_write_buffer = [0; 8 * 1024];
        // let tls_config = TlsConfig::new(
        //     seed,
        //     &mut tls_read_buffer,
        //     &mut tls_write_buffer,
        //     TlsVerify::None,
        // );
        // let mut http_client = HttpClient::new_with_tls(&tcp_client, &dns, tls_config);

        let mut http_client = HttpClient::new(&tcp_client, &dns);
        let mut request = http_client
            .request(Method::GET, IMAGE_URLS[image_index])
            .await
            .unwrap();

        let response = request.send(&mut rx_buffer).await.unwrap();

        println!("Content-length: {:?}", response.content_length);

        let mut png = PngReader::new();
        let mut reader = response.body().reader();

        let mut image_x: u32 = 0;
        let mut image_y: u32 = 0;
        let mut image_header: Option<ImageHeader> = None;

        let mut buf = [0; 2048];
        let mut total_bytes_read = 0;
        loop {
            let start = rtc.get_time_us();
            let n = reader.read(&mut buf).await.unwrap();
            if n == 0 {
                break;
            }
            total_bytes_read += n;
            let duration = rtc.get_time_us() - start;
            println!(
                "Received {} bytes in {}us (total {})",
                n, duration, total_bytes_read
            );
            let start = rtc.get_time_us();
            let mut num_drawn = 0;
            let flow = png.process_data::<()>(&buf[..n], |event| {
                match event {
                    inflater::Event::ImageHeader(header) => {
                        println!("Image header: {:?}", header);
                        image_header = Some(header);

                        display.clear(display::BACKGROUND).unwrap();
                        display::flush(&mut display).unwrap();
                    }
                    inflater::Event::End => {
                        println!("Image end");
                    }
                    inflater::Event::ImageData(pixels) => {
                        let image_header = image_header.as_ref().expect("no header!");

                        // Assuming 8-bit grayscale, no filtering, no interlacing

                        let mut i = 0;
                        while i < pixels.len() {
                            let display_x = image_x as i32 - image_offset_x as i32;
                            let display_y = image_y as i32 - image_offset_y as i32;

                            if display_y >= 0
                                && display_x >= 0
                                && display_x < display_width as i32
                                && image_x < image_header.width
                            {
                                let n = core::cmp::min(
                                    display_width - display_x as u32,
                                    core::cmp::min(
                                        (pixels.len() - i) as u32,
                                        image_header.width - image_x,
                                    ),
                                );
                                display::write_row(
                                    &mut display,
                                    display_x as u32,
                                    display_y as u32,
                                    n,
                                    pixels[i..i + n as usize]
                                        .iter()
                                        .copied()
                                        .map(|pixel| Gray8::new(pixel).into()),
                                )
                                .unwrap();
                                i += n as usize;
                                num_drawn += n;
                                image_x += n as u32;
                            } else {
                                i += 1;
                                image_x += 1;
                            }

                            // FIXME: Logically we shouldn't need the +1 here. But without it the
                            // image renders with a off-by-one error. What's going on?
                            if image_x == image_header.width + 1 {
                                image_x = 0;
                                image_y += 1;
                            }

                            let display_y = image_y as i32 - image_offset_y as i32;
                            if display_y == display_height as i32 {
                                println!("End of display");
                                return ControlFlow::Break(());
                            }
                        }
                    }
                }
                ControlFlow::Continue(())
            });
            let duration = rtc.get_time_us() - start;
            println!("Processed in {}us (drawn {} pixels)", duration, num_drawn);
            display::flush(&mut display).unwrap();
            if let ControlFlow::Break(_) = flow {
                break;
            }
        }

        // Disconnect
        drop(request);
        drop(http_client);

        // Decide what to do next: scroll horizontally, scroll vertically, or next image
        let image_header = image_header.as_ref().expect("no header!");
        if image_offset_x + display_width < image_header.width {
            // scroll right
            image_offset_x = core::cmp::min(
                image_header.width - display_width,
                image_offset_x + display_width * 3 / 2,
            );
        } else if image_offset_y + display_height < image_header.height {
            // back to left edge, scroll vertically
            image_offset_x = 0;
            image_offset_y = core::cmp::min(
                image_header.height - display_height,
                image_offset_y + display_height,
            );
        } else {
            // next image
            image_offset_x = 0;
            image_offset_y = 0;
            image_index = (image_index + 1) % IMAGE_URLS.len();
        }

        // wait for button press
        loop {
            let _ = input.wait_for_any_edge().await;
            if input.is_high().unwrap() {
                break;
            }
        }
    }
}

// TODO: something like this should be in `incremental-png` itself
struct PngReader {
    dechunker: Dechunker,
    sd: StreamDecoder,
    inflater: Inflater<2048>,
}

impl PngReader {
    fn new() -> Self {
        Self {
            dechunker: Dechunker::new(),
            sd: StreamDecoder::new(),
            inflater: Inflater::new(),
        }
    }

    fn process_data<B>(
        &mut self,
        mut input: &[u8],
        mut block: impl FnMut(inflater::Event) -> ControlFlow<B>,
    ) -> ControlFlow<B> {
        while !input.is_empty() {
            let (consumed, mut dc_event) = self.dechunker.update(&input).unwrap();

            while let Some(e) = dc_event {
                let (leftover, mut sd_event) = self.sd.update(e).unwrap();

                while let Some(e) = sd_event {
                    let (leftover, i_event) = self.inflater.update(e).unwrap();

                    if let Some(e) = i_event {
                        block(e)?;
                    }

                    sd_event = leftover;
                }

                dc_event = leftover;
            }

            input = &input[consumed..];
        }
        ControlFlow::Continue(())
    }
}

#[embassy_executor::task]
async fn connection_wifi(mut controller: WifiController<'static>) {
    println!("Start connect with wifi (SSID: {:?}) task", SSID);
    loop {
        if matches!(esp_wifi::wifi::get_wifi_state(), WifiState::StaConnected) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.into(),
                password: PASSWORD.into(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}
