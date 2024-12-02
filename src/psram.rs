#[link_section = ".data"]
#[inline(never)]
pub fn detect_psram(qmi: &rp235x_hal::pac::QMI) -> u32 {
    critical_section::with(|_cs| {
        // Try and read the PSRAM ID via direct_csr.
        qmi.direct_csr().write(|w| unsafe {
            w.clkdiv().bits(30);
            w.en().set_bit();
            w
        });

        // Need to poll for the cooldown on the last XIP transfer to expire
        // (via direct-mode BUSY flag) before it is safe to perform the first
        // direct-mode operation
        while qmi.direct_csr().read().busy().bit_is_set() {
            rp235x_hal::arch::nop();
        }

        qmi.direct_csr().modify(|_r, w| w.assert_cs1n().set_bit());

        // Transmit as quad.
        qmi.direct_tx().write(|w| unsafe {
            w.oe().set_bit();
            w.iwidth().q();
            w.data().bits(0xf5);
            w
        });

        while qmi.direct_csr().read().busy().bit_is_set() {
            rp235x_hal::arch::nop();
        }

        let _ = qmi.direct_rx().read().bits();

        qmi.direct_csr().modify(|_, w| {
            w.assert_cs1n().clear_bit();
            w
        });

        qmi.direct_csr().modify(|_, w| {
            w.assert_cs1n().set_bit();
            w
        });

        let mut kgd: u32 = 0;
        let mut eid: u32 = 0;
        for i in 0usize..7 {
            if i == 0 {
                qmi.direct_tx().write(|w| unsafe { w.bits(0x9f) });
            } else {
                qmi.direct_tx().write(|w| unsafe { w.bits(0xff) });
            }

            while qmi.direct_csr().read().txempty().bit_is_clear() {
                rp235x_hal::arch::nop();
            }

            while qmi.direct_csr().read().busy().bit_is_set() {
                rp235x_hal::arch::nop();
            }

            if i == 5 {
                kgd = qmi.direct_rx().read().bits();
            } else if i == 6 {
                eid = qmi.direct_rx().read().bits();
            } else {
                let _ = qmi.direct_rx().read().bits();
            }
        }

        qmi.direct_csr().modify(|_, w| {
            w.assert_cs1n().clear_bit();
            w.en().clear_bit();
            w
        });
        let mut param_size: u32 = 0;
        if kgd == 0x5D {
            param_size = 1024 * 1024;
            let size_id = eid >> 5;
            if eid == 0x26 || size_id == 2 {
                param_size *= 8;
            } else if size_id == 0 {
                param_size *= 2;
            } else if size_id == 1 {
                param_size *= 4;
            }
        }
        param_size
    })
}

#[link_section = ".data"]
#[inline(never)]
pub fn psram_init(
    clock_hz: u32,
    qmi: &rp235x_hal::pac::QMI,
    xip: &rp235x_hal::pac::XIP_CTRL,
) -> u32 {
    let psram_size = detect_psram(qmi);

    if psram_size == 0 {
        return 0;
    }

    // Set PSRAM timing for APS6404
    //
    // Using an rxdelay equal to the divisor isn't enough when running the APS6404 close to 133MHz.
    // So: don't allow running at divisor 1 above 100MHz (because delay of 2 would be too late),
    // and add an extra 1 to the rxdelay if the divided clock is > 100MHz (i.e. sys clock > 200MHz).
    const MAX_PSRAM_FREQ: u32 = 133_000_000;

    let mut divisor: u32 = (clock_hz + MAX_PSRAM_FREQ - 1) / MAX_PSRAM_FREQ;
    if divisor == 1 && clock_hz > 100_000_000 {
        divisor = 2;
    }
    let mut rxdelay: u32 = divisor;
    if clock_hz / divisor > 100_000_000 {
        rxdelay += 1;
    }

    // - Max select must be <= 8us.  The value is given in multiples of 64 system clocks.
    // - Min deselect must be >= 18ns.  The value is given in system clock cycles - ceil(divisor / 2).
    let clock_period_fs: u64 = 1_000_000_000_000_000_u64 / u64::from(clock_hz);
    let max_select: u8 = ((125 * 1_000_000) / clock_period_fs) as u8;
    let min_deselect: u32 = ((18 * 1_000_000 + (clock_period_fs - 1)) / clock_period_fs
        - u64::from(divisor + 1) / 2) as u32;

    qmi.direct_csr().write(|w| unsafe {
        w.clkdiv().bits(10);
        w.en().set_bit();
        w.auto_cs1n().set_bit();
        w
    });

    while qmi.direct_csr().read().busy().bit_is_set() {
        rp235x_hal::arch::nop();
    }

    qmi.direct_tx().write(|w| unsafe {
        w.nopush().set_bit();
        w.bits(0x35);
        w
    });

    while qmi.direct_csr().read().busy().bit_is_set() {
        rp235x_hal::arch::nop();
    }

    qmi.m1_timing().write(|w| unsafe {
        w.cooldown().bits(1);
        w.pagebreak()._1024();
        w.max_select().bits(max_select as u8);
        w.min_deselect().bits(min_deselect as u8);
        w.rxdelay().bits(rxdelay as u8);
        w.clkdiv().bits(divisor as u8);
        w
    });

    // // Set PSRAM commands and formats
    qmi.m1_rfmt().write(|w| {
        w.prefix_width().q();
        w.addr_width().q();
        w.suffix_width().q();
        w.dummy_width().q();
        w.data_width().q();
        w.prefix_len()._8();
        w.dummy_len()._24();
        w
    });

    qmi.m1_rcmd().write(|w| unsafe { w.bits(0xEB) });

    qmi.m1_wfmt().write(|w| {
        w.prefix_width().q();
        w.addr_width().q();
        w.suffix_width().q();
        w.dummy_width().q();
        w.data_width().q();
        w.prefix_len()._8();
        w
    });

    qmi.m1_wcmd().write(|w| unsafe { w.bits(0x38) });

    // Disable direct mode
    qmi.direct_csr().write(|w| unsafe { w.bits(0) });

    // Enable writes to PSRAM
    xip.ctrl().modify(|_, w| w.writable_m1().set_bit());
    psram_size
}
