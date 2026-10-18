//! Нативный транспорт прямого Bluetooth RFCOMM сокета Linux (AF_BLUETOOTH).

use std::io::{self, Read};
use std::os::raw::{c_int, c_void};
use std::time::Duration;

/// Символьная структура адреса сокета Bluetooth RFCOMM (sockaddr_rc) в Linux.
#[repr(C)]
struct SockAddrRc {
    rc_family: u16,
    rc_bdaddr: [u8; 6],
    rc_channel: u8,
}

/// Поток прямого соединения с радиомодулем Bluetooth Classic без файлов в /dev.
pub struct BtRfcommStream {
    fd: c_int,
}

impl BtRfcommStream {
    /// Устанавливает прямое RFCOMM соединение с целевым MAC-адресом с неблокирующим таймаутом.
    pub fn connect(mac_str: &str, channel: u8, timeout: Duration) -> io::Result<Self> {
        let parts: Vec<&str> = mac_str.split(':').collect();
        if parts.len() != 6 {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "Некорректный формат MAC-адреса"));
        }

        let mut bdaddr = [0u8; 6];
        for (i, part) in parts.iter().enumerate() {
            let byte = u8::from_str_radix(part, 16)
                .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
            bdaddr[5 - i] = byte;
        }

        // AF_BLUETOOTH = 31, SOCK_STREAM = 1, BTPROTO_RFCOMM = 3
        let fd = unsafe { libc::socket(31, 1, 3) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }

        let local_addr = SockAddrRc {
            rc_family: 31,
            rc_bdaddr: [0; 6],
            rc_channel: 0,
        };
        let bind_ret = unsafe {
            libc::bind(
                fd,
                &local_addr as *const _ as *const libc::sockaddr,
                std::mem::size_of::<SockAddrRc>() as libc::socklen_t,
            )
        };
        if bind_ret < 0 {
            let err = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            return Err(err);
        }

        // Перевод сокета в неблокирующий режим для отсечения зависания системного connect
        let flags = unsafe { libc::fcntl(fd, libc::F_GETFL, 0) };
        if flags < 0 {
            let err = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            return Err(err);
        }
        if unsafe { libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) } < 0 {
            let err = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            return Err(err);
        }

        let remote_addr = SockAddrRc {
            rc_family: 31,
            rc_bdaddr: bdaddr,
            rc_channel: channel,
        };

        let ret = unsafe {
            libc::connect(
                fd,
                &remote_addr as *const _ as *const libc::sockaddr,
                std::mem::size_of::<SockAddrRc>() as libc::socklen_t,
            )
        };

        if ret < 0 {
            let err = io::Error::last_os_error();
            if err.raw_os_error() != Some(libc::EINPROGRESS) {
                unsafe { libc::close(fd) };
                return Err(err);
            }

            // Ожидание завершения подключения строго в пределах переданного таймаута
            let mut pfd = libc::pollfd {
                fd,
                events: libc::POLLOUT,
                revents: 0,
            };

            let timeout_ms = timeout.as_millis().clamp(10, 5000) as libc::c_int;
            let poll_ret = unsafe { libc::poll(&mut pfd, 1, timeout_ms) };

            if poll_ret <= 0 {
                unsafe { libc::close(fd) };
                return Err(io::ErrorKind::TimedOut.into());
            }

            let mut sock_err: libc::c_int = 0;
            let mut err_len = std::mem::size_of::<libc::c_int>() as libc::socklen_t;
            let opt_ret = unsafe {
                libc::getsockopt(
                    fd,
                    libc::SOL_SOCKET,
                    libc::SO_ERROR,
                    &mut sock_err as *mut _ as *mut c_void,
                    &mut err_len,
                )
            };

            if opt_ret < 0 || sock_err != 0 {
                unsafe { libc::close(fd) };
                let final_err = if sock_err != 0 {
                    io::Error::from_raw_os_error(sock_err)
                } else {
                    io::Error::last_os_error()
                };
                return Err(final_err);
            }
        }

        // Возврат сокета в блокирующий режим для чтения данных
        if unsafe { libc::fcntl(fd, libc::F_SETFL, flags) } < 0 {
            let err = io::Error::last_os_error();
            unsafe { libc::close(fd) };
            return Err(err);
        }

        let tv = libc::timeval {
            tv_sec: 0,
            tv_usec: 60_000, // 60 мс на чтение порции кадра
        };
        unsafe {
            libc::setsockopt(
                fd,
                1,  // SOL_SOCKET
                20, // SO_RCVTIMEO
                &tv as *const _ as *const c_void,
                std::mem::size_of::<libc::timeval>() as libc::socklen_t,
            );
        }

        Ok(Self { fd })
    }
}

impl Read for BtRfcommStream {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let bytes = unsafe {
            libc::read(self.fd, buf.as_mut_ptr() as *mut c_void, buf.len())
        };
        if bytes < 0 {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(libc::EAGAIN) || err.raw_os_error() == Some(libc::EWOULDBLOCK) {
                return Err(io::ErrorKind::TimedOut.into());
            }
            return Err(err);
        }
        Ok(bytes as usize)
    }
}

impl Drop for BtRfcommStream {
    fn drop(&mut self) {
        unsafe {
            libc::shutdown(self.fd, 2);
            libc::close(self.fd);
        }
    }
}