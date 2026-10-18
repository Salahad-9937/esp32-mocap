//! Абстракция последовательного порта и кроссплатформенный транспорт.

use crate::error::{MocapError, Result};
use std::time::Duration;

/// Трейт абстрактного байтового потока для чтения данных.
pub trait ByteStream: Send {
    /// Считывает байты в предоставленный буфер, возвращая количество прочитанных байт.
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;
}

/// Реализация транспорта последовательного порта (USB-UART / Bluetooth Virtual COM).
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
}

impl SerialTransport {
    /// Открывает последовательный порт с заданным именем и скоростью передачи бод.
    pub fn open(port_name: &str, baud_rate: u32) -> Result<Self> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(20))
            .open()?;
        Ok(Self { port })
    }

    /// Сканирует операционную систему на наличие доступных последовательных портов.
    pub fn available_ports() -> Result<Vec<String>> {
        let ports = serialport::available_ports()?;
        Ok(ports.into_iter().map(|p| p.port_name).collect())
    }
}

impl ByteStream for SerialTransport {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        match self.port.read(buffer) {
            Ok(count) => Ok(count),
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(MocapError::Transport(e.to_string())),
        }
    }
}