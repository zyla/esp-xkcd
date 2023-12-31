# `esp-xkcd`

Embedded Rust XKCD viewer. Displays a random XKCD on a LCD or OLED display.

It is written in Rust, using the `no_std` library, in adherence to the principles described in the [ESP-RS book](https://esp-rs.github.io/book/writing-your-own-application/nostd.html). This repository is based on [esp-jokes-rs](https://github.com/jborkowski/esp-jokes-rs).

What is challenging about this is that the image we're displaying might not fit in RAM (400K on `esp32c3`, but less is available for our code), and sometimes neither does the input file. The [`incremental-png`](https://github.com/zyla/incremental-png) library was created to address this. It's focused in incremental decoding and minimizing memory usage.

![image](https://github.com/zyla/esp-xkcd/assets/1410069/c6f6e7a0-8577-4229-bb78-c4de78236b0d)

_`esp-xkcd` running on `esp32c2` with SSD1306 display, showing [XKCD #2846](https://xkcd.com/2846/)  
Original image license: [CC BY-NC 2.5](https://xkcd.com/license.html)_

## Features

- Displays XKCD image
- Written in Rust, showcasing how to use the `no_std` library for ESP32 development.
- Supports various display drivers, including ST7735 and SSD1306

## Supported hardware configurations

Hardware options are selected using Cargo features. Some are selected by default. To change the configuration pass `--no-default-features --features <your_features>`.

Boards:

- `esp32c3`
- `esp32c6`

Displays:

- `display-st7735`
- `display-ssd1306`

To build in the default mode (ESP32C3, ST7735), simply run:

   ```bash
   cargo run
   ```

For example, to build on ESP32C6 with SSD1306 display, use the following command:

   ```bash
   cargo run --no-default-features --features esp32c6,display-ssd1306
   ```

## Getting Started

To get started with this project, you will need:

- An ESP32 development board (See [Supported hardware configurations](#supported-hardware-configurations) above for which boards are supported)
- An OLED or LCD display connected to your ESP32.
- Rust and the associated toolchain installed on your development machine.

Follow these steps to set up and run the project:

1. Clone this repository to your development machine.

   ```bash
   git clone https://github.com/jborkowski/esp-jokes-rs.git
   ```

2. Change the working directory to the project folder.

   ```bash
   cd esp-jokes-rs
   ```

3. Build the project using Cargo, the Rust package manager.

   ```bash
   cargo build
   ```

4. Flash the resulting binary to your ESP32 board.

   ```bash
   cargo run
   ```
   **Note**: You might need additional options here depending on your hardware configuration, see [Supported hardware configurations](#supported-hardware-configurations).

5. Once the flashing process is complete, ESP board will connect to WiFi and display the image. Enjoy!

## Project Structure

- `src/` contains the Rust source code for the project.
- `Cargo.toml` is the configuration file for the project, where dependencies and build options are specified.

## Contributing

We welcome contributions to this project. If you have ideas for more improvements or want to add features, feel free to open an issue or a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- This project is based on the Wokwi example project, and we are grateful for the inspiration it provided.

## Contributors

- [Jonatan Borkowski](https://github.com/jborkowski)
- [Maciej Bielecki](https://github.com/zyla)

## Special Thanks

Special thanks to the ESP32 community, the Rust programming language developers, and everyone who contributed to the libraries and tools used in this project.
