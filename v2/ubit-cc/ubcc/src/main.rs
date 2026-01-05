#![no_main]
#![no_std]

use defmt_rtt as _;
use microbit::{
    hal::gpio::{
        p0,
        p1,
        Output,
        Pin,
        Port,
        PushPull
    },
};
use panic_halt as _;

use core::{
    str//,
//    fmt::Write
};
//use embedded_io::Read;
use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use microbit::hal::{
    clocks::Clocks,
    timer::Timer,
};
use rubble::beacon::Beacon;
use rubble::link::{ad_structure::AdStructure, CompanyId, MIN_PDU_BUF};
use rubble_nrf5x::radio::{
    BleRadio,
    PacketBuffer
};
use rubble_nrf5x::utils::get_device_address;

//mod serial_setup;
//use serial_setup::UartePort;

#[entry]
fn main() -> ! {
    static mut BLE_TX_BUF: PacketBuffer = [0; MIN_PDU_BUF];
    static mut BLE_RX_BUF: PacketBuffer = [0; MIN_PDU_BUF];

    if let Some(p) = microbit::Peripherals::take() {
/*
        let mut serial = {
            let serial = uarte::Uarte::new(
                board.UARTE0,
                board.uart.into(),
                Parity::EXCLUDED,
                Baudrate::BAUD115200,
            );
            UartePort::new(serial)
        };
        /* Fire up receiving task */
        //uart0.tasks_startrx.write(|w| unsafe { w.bits(1) });
        defmt::info!("Starting echo loop");

        /* Endless loop */
        loop {
            defmt::info!("Hello World:\r\n");
            let mut input = [0];
            serial.read_exact(&mut input).unwrap();
            defmt::info!("You said: {}\r\n", input[0] as char);
        }*/

        /* Configure RX and TX pins accordingly */
        //p.GPIO.pin_cnf[gpio::UART_RX].write(|w| w.pull().pullup().dir().output());
        //p.GPIO.pin_cnf[UART_TX].write(|w| w.pull().disabled().dir().input());


        defmt::info!("Setting up UART... ");
        let uart0 = p.UART0;
        let p0_parts = p0::Parts::new(p.P0);
        let p1_parts = p1::Parts::new(p.P1);
        let tx = p0_parts.p0_06.degrade();
        let rx = p1_parts.p1_08.degrade();
        /* Tell UART which pins to use for sending and receiving */
        uart0.psel.txd.write(|w| unsafe { w.bits(tx.pin() as u32)});
        uart0.psel.rxd.write(|w| unsafe { w.bits(rx.pin() as u32)});

        /* Set a typical baud rate of 115200 */
        uart0.baudrate.write(|w| w.baudrate().baud115200());

        /* Enable UART function */
        uart0.enable.write(|w| w.enable().enabled());

        /* Print a nice hello message */
        let _ = write_uart0(&uart0, "Please type characters to echo:\r\n");

        defmt::info!("Done\n");

        /* Fire up receiving task */
        uart0.tasks_startrx.write(|w| unsafe { w.bits(1) });

        /* Endless loop */
        loop {
            /* Busy wait for reception of data */
            while uart0.events_rxdrdy.read().bits() == 0 {}

            /* We're going to pick up the data soon, let's signal the buffer is already waiting for
             * more data */
            uart0.events_rxdrdy.write(|w| unsafe { w.bits(0) });

            /* Read one 8bit value */
            let c = uart0.rxd.read().bits() as u8;
            defmt::info!("rx: {}", defmt::Display2Format(&c));
            /* What comes in must go out, we don't care what it is */
            //let _ = write_uart0(&uart0, unsafe { str::from_utf8_unchecked(&[c; 1]) });
        }
    }

    if let Some(p) = microbit::Peripherals::take() {
        // On reset, the internal high frequency clock is already used, but we
        // also need to switch to the external HF oscillator. This is needed
        // for Bluetooth to work.
        let _clocks = Clocks::new(p.CLOCK).enable_ext_hfosc();

        // Determine device address
        let device_address = get_device_address();
        defmt::info!("Device address: {}", defmt::Display2Format(&device_address));

        // Rubble currently requires an RX buffer even though the radio is only used as a TX-only beacon.
        let mut radio = BleRadio::new(p.RADIO, &p.FICR, BLE_TX_BUF, BLE_RX_BUF);

        let mut timer = Timer::new(p.TIMER0);

        loop {
            // Broadcast local name
            let local_name = "Rusty microbit";
            //defmt::info!("Local name: {}", local_name);
            let beacon = Beacon::new(
                device_address,
                &[AdStructure::CompleteLocalName(local_name)],
            )
            .unwrap();
            beacon.broadcast(&mut radio);
            timer.delay_ms(500_u32);

            // Broadcast data
            let data = "Hello world";
            //defmt::info!("Data: {}", data);
            let beacon = Beacon::new(
                device_address,
                &[AdStructure::ManufacturerSpecificData {
                    company_identifier: CompanyId::from_raw(0xffff),
                    payload: data.as_bytes(),
                }],
            )
            .unwrap();
            //beacon.broadcast(&mut radio);
            //timer.delay_ms(500_u32);
        }
    }

    loop {}
}

fn write_uart0(uart0: &microbit::pac::UART0, s: &str) -> core::fmt::Result {
    /* Start UART sender */
    uart0.tasks_starttx.write(|w| unsafe { w.bits(1) });

    for c in s.as_bytes() {
        /* Write the current character to the output register */
        uart0.txd.write(|w| unsafe { w.bits(u32::from(*c)) });

        /* Wait until the UART is clear to send */
        while uart0.events_txdrdy.read().bits() == 0 {}

        /* And then reset it for the next round */
        uart0.events_txdrdy.write(|w| unsafe { w.bits(0) });
    }

    /* Stop UART sender */
    uart0.tasks_stoptx.write(|w| unsafe { w.bits(1) });
    Ok(())
}
