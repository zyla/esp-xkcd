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
    use embedded_graphics::pixelcolor::Rgb565;
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

    #[cfg(feature = "display-st7735")]
    let mut display: DISPLAY = {
        use hal::{Delay, Spi};

        let miso = io.pins.gpio6.into_push_pull_output(); // A0
        let rst = io.pins.gpio7.into_push_pull_output();

        let spi: SPI = Spi::new(
            peripherals.SPI2,
            io.pins.gpio1,
            io.pins.gpio2,                         // sda
            io.pins.gpio0.into_push_pull_output(), // dc not connected
            io.pins.gpio8,
            60u32.MHz(),
            hal::spi::SpiMode::Mode0,
            &mut system.peripheral_clock_control,
            &clocks,
        );

        let mut display = st7735_lcd::ST7735::new(spi, miso, rst, true, false, 160, 128);

        let mut delay = Delay::new(&clocks);
        display.init(&mut delay).unwrap();
        display
            .set_orientation(&st7735_lcd::Orientation::Landscape)
            .unwrap();
        display.set_offset(0, 0);
        display
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

    display.clear(display::BACKGROUND).unwrap();
    display::flush(&mut display).unwrap();

    spawner.spawn(connection_wifi(controller)).ok();
    spawner.spawn(net_task(stack)).ok();
    spawner.spawn(task(input, stack, seed.into(), display)).ok();
}

#[embassy_executor::task]
async fn task(
    mut input: Gpio9<Input<PullUp>>,
    stack: &'static Stack<WifiDevice<'static>>,
    _seed: u64,
    mut display: DISPLAY<'static>,
) {
    let mut rx_buffer = [0; 512];
    let client_state = TcpClientState::<1, 512, 512>::new();
    let tcp_client = TcpClient::new(stack, &client_state);
    let dns = DnsSocket::new(stack);

    const IMAGE_URLS: &[&str] = &[
        "http://imgs.xkcd.com/comics/the_universal_label.png",
        "http://imgs.xkcd.com/comics/journal_4.png",
        "http://imgs.xkcd.com/comics/daylight_saving_choice.png",
        // biggest grayscale image - 549K
        "http://imgs.xkcd.com/comics/the_pace_of_modern_life.png",
        // non-text
        "http://imgs.xkcd.com/comics/to_be_wanted.png",
        // Doesn't work, we only handle grayscale images for now
        // "http://imgs.xkcd.com/comics/dendrochronology.png",

        // Crashes with "buffer error" from incremental-png :(
        // "http://imgs.xkcd.com/comics/depth.png",
    ];

    let mut image_index = 0;

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
        image_index = (image_index + 1) % IMAGE_URLS.len();

        let response = request.send(&mut rx_buffer).await.unwrap();

        println!("Content-length: {:?}", response.content_length);

        let mut png = PngReader::new();
        let mut reader = response.body().reader();

        let display_width = display.dimensions().0 as u32;
        let display_height = display.dimensions().1 as u32;
        let mut x: u32 = 0;
        let mut y: u32 = 0;
        let mut image_header: Option<ImageHeader> = None;

        let mut buf = [0; 1024];
        let mut total_bytes_read = 0;
        loop {
            let n = reader.read(&mut buf).await.unwrap();
            if n == 0 {
                break;
            }
            total_bytes_read += n;
            println!("Received {} bytes (total {})", n, total_bytes_read);
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
                        println!("Decoded {} pixels", pixels.len());

                        // Assuming 8-bit grayscale, no filtering, no interlacing

                        for pixel in pixels.iter().copied() {
                            if x < display_width {
                                display.set_pixel(x, y, pixel < 128);
                            }
                            x += 1;

                            // FIXME: Logically we shouldn't need the +1 here. But without it the
                            // image renders with a off-by-one error. What's going on?
                            if x == image_header.width + 1 {
                                x = 0;
                                y += 1;
                            }
                            if y == display_height {
                                println!("End of display");
                                return ControlFlow::Break(());
                            }
                        }
                    }
                }
                ControlFlow::Continue(())
            });
            display::flush(&mut display).unwrap();
            if let ControlFlow::Break(_) = flow {
                break;
            }
        }

        // Disconnect
        drop(request);
        drop(http_client);

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
    inflater: Inflater<256>,
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
