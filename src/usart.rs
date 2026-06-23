//! USART

use core::fmt;
use core::marker::PhantomData;

use embedded_dma::{ReadBuffer, WriteBuffer};
use embedded_hal_nb::nb::{Error as NbError, Result as NbResult};
use embedded_hal_nb::serial::{ErrorKind, ErrorType, Read, Write};
use mik32_pac::usart_0::RegisterBlock;
use mik32_pac::{Peripherals, Usart0, Usart1};

use crate::dma::{Channel as DmaChannel, ChannelId as DmaChannelId, Error as DmaError};
use crate::gpio::{Func2Mode, Func3Mode, Pin};
use crate::rcc::system_clock;

const DEFAULT_INIT_TIMEOUT: u32 = 100_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WordLength {
    DataBits7,
    DataBits8,
    DataBits9,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DuplexMode {
    Half,
    Full,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncMode {
    Async,
    Sync,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockPolarity {
    IdleLow,
    IdleHigh,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockPhase {
    FirstEdge,
    SecondEdge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Even,
    Odd,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    Stop1,
    Stop2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaConfig {
    None,
    Tx,
    Rx,
    TxRx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitOrder {
    LsbFirst,
    MsbFirst,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowControl {
    None,
    Rts,
    Cts,
    RtsCts,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtsMode {
    AlwaysEnabled,
    Modem,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModemConfig {
    pub dtr: bool,
}

impl ModemConfig {
    pub const fn disabled() -> Self {
        Self { dtr: false }
    }

    pub const fn dtr(mut self, enabled: bool) -> Self {
        self.dtr = enabled;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub baudrate: u32,
    pub word_length: WordLength,
    pub parity: Parity,
    pub stop_bits: StopBits,
    pub duplex_mode: DuplexMode,
    pub sync_mode: SyncMode,
    pub clock_polarity: ClockPolarity,
    pub clock_phase: ClockPhase,
    pub clock_last_bit: bool,
    pub bit_order: BitOrder,
    pub data_inversion: bool,
    pub tx_inversion: bool,
    pub rx_inversion: bool,
    pub swap_pins: bool,
    pub loopback: bool,
    pub tx_break: bool,
    pub overwrite: bool,
    pub flow_control: FlowControl,
    pub rts_mode: RtsMode,
    pub modem: ModemConfig,
    pub dma: DmaConfig,
    pub init_timeout: u32,
}

impl Config {
    pub const fn default() -> Self {
        Self {
            baudrate: 115_200,
            word_length: WordLength::DataBits8,
            parity: Parity::None,
            stop_bits: StopBits::Stop1,
            duplex_mode: DuplexMode::Full,
            sync_mode: SyncMode::Async,
            clock_polarity: ClockPolarity::IdleLow,
            clock_phase: ClockPhase::FirstEdge,
            clock_last_bit: false,
            bit_order: BitOrder::LsbFirst,
            data_inversion: false,
            tx_inversion: false,
            rx_inversion: false,
            swap_pins: false,
            loopback: false,
            tx_break: false,
            overwrite: false,
            flow_control: FlowControl::None,
            rts_mode: RtsMode::AlwaysEnabled,
            modem: ModemConfig::disabled(),
            dma: DmaConfig::None,
            init_timeout: DEFAULT_INIT_TIMEOUT,
        }
    }

    pub const fn baudrate(mut self, baudrate: u32) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub const fn word_length(mut self, word_length: WordLength) -> Self {
        self.word_length = word_length;
        self
    }

    pub const fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    pub const fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }

    pub const fn duplex_mode(mut self, duplex_mode: DuplexMode) -> Self {
        self.duplex_mode = duplex_mode;
        self
    }

    pub const fn sync_mode(mut self, sync_mode: SyncMode) -> Self {
        self.sync_mode = sync_mode;
        self
    }

    pub const fn clock_polarity(mut self, polarity: ClockPolarity) -> Self {
        self.clock_polarity = polarity;
        self
    }

    pub const fn clock_phase(mut self, phase: ClockPhase) -> Self {
        self.clock_phase = phase;
        self
    }

    pub const fn clock_last_bit(mut self, enabled: bool) -> Self {
        self.clock_last_bit = enabled;
        self
    }

    pub const fn bit_order(mut self, bit_order: BitOrder) -> Self {
        self.bit_order = bit_order;
        self
    }

    pub const fn data_inversion(mut self, enabled: bool) -> Self {
        self.data_inversion = enabled;
        self
    }

    pub const fn tx_inversion(mut self, enabled: bool) -> Self {
        self.tx_inversion = enabled;
        self
    }

    pub const fn rx_inversion(mut self, enabled: bool) -> Self {
        self.rx_inversion = enabled;
        self
    }

    pub const fn swap_pins(mut self, enabled: bool) -> Self {
        self.swap_pins = enabled;
        self
    }

    pub const fn loopback(mut self, enabled: bool) -> Self {
        self.loopback = enabled;
        self
    }

    pub const fn tx_break(mut self, enabled: bool) -> Self {
        self.tx_break = enabled;
        self
    }

    pub const fn overwrite(mut self, enabled: bool) -> Self {
        self.overwrite = enabled;
        self
    }

    pub const fn flow_control(mut self, flow_control: FlowControl) -> Self {
        self.flow_control = flow_control;
        self
    }

    pub const fn rts_mode(mut self, mode: RtsMode) -> Self {
        self.rts_mode = mode;
        self
    }

    pub const fn modem(mut self, modem: ModemConfig) -> Self {
        self.modem = modem;
        self
    }

    pub const fn dma(mut self, dma: DmaConfig) -> Self {
        self.dma = dma;
        self
    }

    pub const fn init_timeout(mut self, timeout: u32) -> Self {
        self.init_timeout = timeout;
        self
    }

    pub const fn validate(&self) -> Result<(), ConfigError> {
        if self.baudrate == 0 {
            return Err(ConfigError::ZeroBaudrate);
        }
        if self.init_timeout == 0 {
            return Err(ConfigError::ZeroInitTimeout);
        }

        Ok(())
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::default()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    ZeroBaudrate,
    ZeroInitTimeout,
    BaudrateTooHigh,
    BaudrateTooLow,
    DedicatedConstructorRequired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitErrorKind {
    InvalidConfig(ConfigError),
    PeripheralNotReady,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaTransferError {
    Dma(DmaError),
    InvalidWordLength,
    WordOutOfRange,
}

impl From<DmaError> for DmaTransferError {
    fn from(error: DmaError) -> Self {
        Self::Dma(error)
    }
}

pub struct DmaTransferFailure<PERIPHERAL, CHANNEL, BUFFER> {
    pub error: DmaTransferError,
    pub peripheral: PERIPHERAL,
    pub channel: CHANNEL,
    pub buffer: BUFFER,
}

impl<PERIPHERAL, CHANNEL, BUFFER> DmaTransferFailure<PERIPHERAL, CHANNEL, BUFFER> {
    pub fn into_parts(self) -> (DmaTransferError, PERIPHERAL, CHANNEL, BUFFER) {
        (self.error, self.peripheral, self.channel, self.buffer)
    }
}

impl<PERIPHERAL, CHANNEL, BUFFER> fmt::Debug for DmaTransferFailure<PERIPHERAL, CHANNEL, BUFFER> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("DmaTransferFailure")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

pub struct InitError<UART, TXPIN, RXPIN> {
    pub uart: UART,
    pub pins: (TXPIN, RXPIN),
    pub error: InitErrorKind,
}

impl<UART, TXPIN, RXPIN> InitError<UART, TXPIN, RXPIN> {
    pub fn into_parts(self) -> (UART, (TXPIN, RXPIN), InitErrorKind) {
        (self.uart, self.pins, self.error)
    }
}

impl<UART, TXPIN, RXPIN> fmt::Debug for InitError<UART, TXPIN, RXPIN> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("InitError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

pub struct ModeInitError<UART, PINS> {
    pub uart: UART,
    pub pins: PINS,
    pub error: InitErrorKind,
}

impl<UART, PINS> ModeInitError<UART, PINS> {
    pub fn into_parts(self) -> (UART, PINS, InitErrorKind) {
        (self.uart, self.pins, self.error)
    }
}

impl<UART, PINS> fmt::Debug for ModeInitError<UART, PINS> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ModeInitError")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}

pub trait TxPin<UART> {}
pub trait RxPin<UART> {}
pub trait ClockPin<UART> {}
pub trait HalfDuplexPin<UART> {}
pub trait RtsPin<UART> {}
pub trait CtsPin<UART> {}
pub trait DtrPin<UART> {}
pub trait DsrPin<UART> {}
pub trait DcdPin<UART> {}
pub trait RiPin<UART> {}
pub trait DdisPin<UART> {}

pub trait FlowControlPins<UART> {
    const FLOW_CONTROL: FlowControl;
}

pub struct CtsOnly<CTS>(pub CTS);

impl<UART> FlowControlPins<UART> for () {
    const FLOW_CONTROL: FlowControl = FlowControl::None;
}

impl<UART, RTS> FlowControlPins<UART> for (RTS,)
where
    RTS: RtsPin<UART>,
{
    const FLOW_CONTROL: FlowControl = FlowControl::Rts;
}

impl<UART, RTS, CTS> FlowControlPins<UART> for (RTS, CTS)
where
    RTS: RtsPin<UART>,
    CTS: CtsPin<UART>,
{
    const FLOW_CONTROL: FlowControl = FlowControl::RtsCts;
}

impl<UART, CTS> FlowControlPins<UART> for CtsOnly<CTS>
where
    CTS: CtsPin<UART>,
{
    const FLOW_CONTROL: FlowControl = FlowControl::Cts;
}

pub trait ModemPins<UART> {}

impl<UART> ModemPins<UART> for () {}

impl TxPin<Usart0> for Pin<0, 6, Func2Mode> {}
impl RxPin<Usart0> for Pin<0, 5, Func2Mode> {}
impl CtsPin<Usart0> for Pin<0, 7, Func2Mode> {}
impl RtsPin<Usart0> for Pin<0, 8, Func2Mode> {}
impl DdisPin<Usart0> for Pin<1, 6, Func3Mode> {}
impl DtrPin<Usart0> for Pin<1, 12, Func3Mode> {}
impl DcdPin<Usart0> for Pin<1, 13, Func3Mode> {}
impl DsrPin<Usart0> for Pin<1, 14, Func3Mode> {}
impl RiPin<Usart0> for Pin<1, 15, Func3Mode> {}

impl TxPin<Usart1> for Pin<1, 9, Func2Mode> {}
impl RxPin<Usart1> for Pin<1, 8, Func2Mode> {}
impl CtsPin<Usart1> for Pin<1, 10, Func2Mode> {}
impl RtsPin<Usart1> for Pin<1, 11, Func2Mode> {}
impl ClockPin<Usart0> for Pin<1, 5, Func3Mode> {}
impl ClockPin<Usart1> for Pin<2, 6, Func3Mode> {}
impl DtrPin<Usart1> for Pin<2, 0, Func3Mode> {}
impl DcdPin<Usart1> for Pin<2, 1, Func3Mode> {}
impl DsrPin<Usart1> for Pin<2, 2, Func3Mode> {}
impl RiPin<Usart1> for Pin<2, 3, Func3Mode> {}
impl DdisPin<Usart1> for Pin<2, 7, Func3Mode> {}
impl HalfDuplexPin<Usart0> for Pin<0, 6, Func2Mode> {}
impl HalfDuplexPin<Usart1> for Pin<1, 9, Func2Mode> {}

impl<UART, DTR> ModemPins<UART> for (DTR,) where DTR: DtrPin<UART> {}

impl<UART, DTR, DSR, DCD, RI, DDIS> ModemPins<UART> for (DTR, DSR, DCD, RI, DDIS)
where
    DTR: DtrPin<UART>,
    DSR: DsrPin<UART>,
    DCD: DcdPin<UART>,
    RI: RiPin<UART>,
    DDIS: DdisPin<UART>,
{
}

mod sealed {
    pub trait Sealed {}

    impl Sealed for mik32_pac::Usart0 {}
    impl Sealed for mik32_pac::Usart1 {}
}

pub trait Instance: sealed::Sealed {
    fn ptr() -> *const RegisterBlock;
    fn enable_clock();
    const DMA_REQUEST: u32;
}

impl Instance for Usart0 {
    const DMA_REQUEST: u32 = 0;
    #[inline(always)]
    fn ptr() -> *const RegisterBlock {
        Usart0::ptr()
    }

    #[inline(always)]
    fn enable_clock() {
        let p = unsafe { Peripherals::steal() };
        p.pm.clk_apb_p_set().modify(|_, w| w.uart_0().enable());
    }
}

impl Instance for Usart1 {
    const DMA_REQUEST: u32 = 1;
    #[inline(always)]
    fn ptr() -> *const RegisterBlock {
        Usart1::ptr()
    }

    #[inline(always)]
    fn enable_clock() {
        let p = unsafe { Peripherals::steal() };
        p.pm.clk_apb_p_set().modify(|_, w| w.uart_1().enable());
    }
}

pub struct Serial<UART, TXPIN, RXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
{
    uart: UART,
    pins: (TXPIN, RXPIN),
    word_length: WordLength,
}

pub struct TxOnly<UART, TXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
{
    tx: Tx<UART, TXPIN, Owned>,
}

pub struct RxOnly<UART, RXPIN>
where
    UART: Instance,
    RXPIN: RxPin<UART>,
{
    rx: Rx<UART, RXPIN, Owned>,
}

pub struct FlowControlSerial<UART, TXPIN, RXPIN, FLOWPINS>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
    FLOWPINS: FlowControlPins<UART>,
{
    serial: Serial<UART, TXPIN, RXPIN>,
    flow_pins: FLOWPINS,
}

pub struct ModemSerial<UART, TXPIN, RXPIN, MODEMPINS>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
    MODEMPINS: ModemPins<UART>,
{
    serial: Serial<UART, TXPIN, RXPIN>,
    modem_pins: MODEMPINS,
}

pub struct Owned;
pub struct Borrowed;

pub struct Tx<UART: Instance, TXPIN = (), OWNER = Owned> {
    uart: Option<UART>,
    pin: TXPIN,
    _uart: PhantomData<UART>,
    _owner: PhantomData<OWNER>,
    word_length: WordLength,
}

pub struct Rx<UART: Instance, RXPIN = (), OWNER = Owned> {
    uart: Option<UART>,
    pin: RXPIN,
    _uart: PhantomData<UART>,
    _owner: PhantomData<OWNER>,
    word_length: WordLength,
}

/// USART transmit DMA transfer.
///
/// The transfer owns the transmitter, DMA channel and buffer while DMA is
/// active. This prevents the channel or buffer from being reused before the
/// hardware is finished with them. Use [`TxDmaTransfer::wait`],
/// [`TxDmaTransfer::wait_timeout`] or [`TxDmaTransfer::abort`] to recover the
/// owned parts.
///
/// Dropping this value aborts the DMA channel and restores the USART DMA
/// request bit to the state it had before the transfer was started.
pub struct TxDmaTransfer<UART: Instance, TXPIN, OWNER, CHANNEL: DmaChannelId, BUFFER> {
    tx: Option<Tx<UART, TXPIN, OWNER>>,
    channel: Option<DmaChannel<CHANNEL>>,
    buffer: Option<BUFFER>,
    request_was_enabled: bool,
}

/// USART receive DMA transfer.
///
/// The transfer owns the receiver, DMA channel and buffer while DMA is active.
/// This prevents the channel or buffer from being reused before the hardware
/// is finished with them. Use [`RxDmaTransfer::wait`],
/// [`RxDmaTransfer::wait_timeout`] or [`RxDmaTransfer::abort`] to recover the
/// owned parts.
///
/// Dropping this value aborts the DMA channel and restores the USART DMA
/// request bit to the state it had before the transfer was started.
pub struct RxDmaTransfer<UART: Instance, RXPIN, OWNER, CHANNEL: DmaChannelId, BUFFER> {
    rx: Option<Rx<UART, RXPIN, OWNER>>,
    channel: Option<DmaChannel<CHANNEL>>,
    buffer: Option<BUFFER>,
    request_was_enabled: bool,
}

pub struct SynchronousSerial<UART, TXPIN, RXPIN, CLOCKPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
    CLOCKPIN: ClockPin<UART>,
{
    serial: Serial<UART, TXPIN, RXPIN>,
    clock: CLOCKPIN,
}

pub struct HalfDuplex<UART, PIN>
where
    UART: Instance,
    PIN: HalfDuplexPin<UART>,
{
    uart: UART,
    pin: PIN,
    word_length: WordLength,
}

impl<UART, TXPIN, RXPIN> Serial<UART, TXPIN, RXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
{
    pub fn new(
        uart: UART,
        pins: (TXPIN, RXPIN),
        config: Config,
    ) -> Result<Self, InitError<UART, TXPIN, RXPIN>> {
        if config.duplex_mode != DuplexMode::Full
            || config.sync_mode != SyncMode::Async
            || config.flow_control != FlowControl::None
            || config.modem.dtr
        {
            return Err(InitError {
                uart,
                pins,
                error: InitErrorKind::InvalidConfig(ConfigError::DedicatedConstructorRequired),
            });
        }
        Self::new_configured(uart, pins, config)
    }

    fn new_configured(
        uart: UART,
        pins: (TXPIN, RXPIN),
        config: Config,
    ) -> Result<Self, InitError<UART, TXPIN, RXPIN>> {
        if let Err(error) = config.validate() {
            return Err(InitError {
                uart,
                pins,
                error: InitErrorKind::InvalidConfig(error),
            });
        }

        let baudrate_divisor = match calc_baudrate_divisor(config.baudrate) {
            Ok(divisor) => divisor,
            Err(error) => {
                return Err(InitError {
                    uart,
                    pins,
                    error: InitErrorKind::InvalidConfig(error),
                });
            }
        };

        UART::enable_clock();

        let serial = Self {
            uart,
            pins,
            word_length: config.word_length,
        };
        if let Err(error) = configure_uart::<UART>(config, baudrate_divisor, true, true) {
            serial.regs().control1().modify(|_, w| w.ue().disable());
            let Self {
                uart,
                pins,
                word_length: _,
            } = serial;
            return Err(InitError { uart, pins, error });
        }

        Ok(serial)
    }

    pub fn release(self) -> (UART, (TXPIN, RXPIN)) {
        regs::<UART>().control1().modify(|_, w| w.ue().disable());
        (self.uart, self.pins)
    }

    pub fn split(self) -> (Tx<UART, TXPIN, Owned>, Rx<UART, RXPIN, Borrowed>) {
        let Self {
            uart,
            pins: (tx_pin, rx_pin),
            word_length,
        } = self;

        (
            Tx {
                uart: Some(uart),
                pin: tx_pin,
                _uart: PhantomData,
                _owner: PhantomData,
                word_length,
            },
            Rx {
                uart: None,
                pin: rx_pin,
                _uart: PhantomData,
                _owner: PhantomData,
                word_length,
            },
        )
    }

    pub fn reunite(
        tx: Tx<UART, TXPIN, Owned>,
        rx: Rx<UART, RXPIN, Borrowed>,
    ) -> Serial<UART, TXPIN, RXPIN> {
        let Tx {
            uart,
            pin: tx_pin,
            word_length,
            ..
        } = tx;
        let Rx { pin: rx_pin, .. } = rx;
        Serial {
            uart: uart.expect("split Tx missing USART owner"),
            pins: (tx_pin, rx_pin),
            word_length,
        }
    }

    #[inline(always)]
    fn regs(&self) -> &RegisterBlock {
        unsafe { &*UART::ptr() }
    }
}

impl<UART, TXPIN> TxOnly<UART, TXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
{
    pub fn new(uart: UART, pin: TXPIN, config: Config) -> Result<Self, ModeInitError<UART, TXPIN>> {
        let tx = new_tx_only(uart, pin, config)?;
        Ok(Self { tx })
    }

    pub fn split(self) -> Tx<UART, TXPIN, Owned> {
        self.tx
    }

    pub fn release(self) -> (UART, TXPIN) {
        self.tx.release()
    }
}

impl<UART, RXPIN> RxOnly<UART, RXPIN>
where
    UART: Instance,
    RXPIN: RxPin<UART>,
{
    pub fn new(uart: UART, pin: RXPIN, config: Config) -> Result<Self, ModeInitError<UART, RXPIN>> {
        let rx = new_rx_only(uart, pin, config)?;
        Ok(Self { rx })
    }

    pub fn split(self) -> Rx<UART, RXPIN, Owned> {
        self.rx
    }

    pub fn release(self) -> (UART, RXPIN) {
        self.rx.release()
    }
}

impl<UART, TXPIN, RXPIN, FLOWPINS> FlowControlSerial<UART, TXPIN, RXPIN, FLOWPINS>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
    FLOWPINS: FlowControlPins<UART>,
{
    pub fn new(
        uart: UART,
        pins: (TXPIN, RXPIN, FLOWPINS),
        mut config: Config,
    ) -> Result<Self, ModeInitError<UART, (TXPIN, RXPIN, FLOWPINS)>> {
        config.flow_control = FLOWPINS::FLOW_CONTROL;
        let (tx, rx, flow_pins) = pins;
        match Serial::new_configured(uart, (tx, rx), config) {
            Ok(serial) => Ok(Self { serial, flow_pins }),
            Err(error) => {
                let (uart, (tx, rx), error) = error.into_parts();
                Err(ModeInitError {
                    uart,
                    pins: (tx, rx, flow_pins),
                    error,
                })
            }
        }
    }

    pub fn release(self) -> (UART, (TXPIN, RXPIN, FLOWPINS)) {
        let (uart, (tx, rx)) = self.serial.release();
        (uart, (tx, rx, self.flow_pins))
    }

    pub fn split(self) -> (Tx<UART, TXPIN, Owned>, Rx<UART, RXPIN, Borrowed>, FLOWPINS) {
        let flow_pins = self.flow_pins;
        let (tx, rx) = self.serial.split();
        (tx, rx, flow_pins)
    }
}

impl<UART, TXPIN, RXPIN, MODEMPINS> ModemSerial<UART, TXPIN, RXPIN, MODEMPINS>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
    MODEMPINS: ModemPins<UART>,
{
    pub fn new(
        uart: UART,
        pins: (TXPIN, RXPIN, MODEMPINS),
        config: Config,
    ) -> Result<Self, ModeInitError<UART, (TXPIN, RXPIN, MODEMPINS)>> {
        let (tx, rx, modem_pins) = pins;
        match Serial::new_configured(uart, (tx, rx), config) {
            Ok(serial) => Ok(Self { serial, modem_pins }),
            Err(error) => {
                let (uart, (tx, rx), error) = error.into_parts();
                Err(ModeInitError {
                    uart,
                    pins: (tx, rx, modem_pins),
                    error,
                })
            }
        }
    }

    pub fn set_dtr(&mut self, ready: bool) {
        regs::<UART>().modem().modify(|_, w| w.dtr().bit(ready));
    }

    pub fn is_dtr_ready(&self) -> bool {
        regs::<UART>().modem().read().dtr().bit_is_set()
    }

    pub fn is_dsr_high(&self) -> bool {
        regs::<UART>().modem().read().dsr().bit_is_set()
    }

    pub fn dsr_changed(&self) -> bool {
        regs::<UART>().modem().read().dsrif().bit_is_set()
    }

    pub fn clear_dsr_changed(&mut self) {
        regs::<UART>()
            .modem()
            .write(|w| w.dsrif().clear_bit_by_one());
    }

    pub fn is_dcd_high(&self) -> bool {
        regs::<UART>().modem().read().dcd().bit_is_set()
    }

    pub fn dcd_changed(&self) -> bool {
        regs::<UART>().modem().read().dcdif().bit_is_set()
    }

    pub fn clear_dcd_changed(&mut self) {
        regs::<UART>()
            .modem()
            .write(|w| w.dcdif().clear_bit_by_one());
    }

    pub fn is_ri_high(&self) -> bool {
        regs::<UART>().modem().read().ri().bit_is_set()
    }

    pub fn ri_changed(&self) -> bool {
        regs::<UART>().modem().read().riif().bit_is_set()
    }

    pub fn clear_ri_changed(&mut self) {
        regs::<UART>()
            .modem()
            .write(|w| w.riif().clear_bit_by_one());
    }

    pub fn release(self) -> (UART, (TXPIN, RXPIN, MODEMPINS)) {
        let (uart, (tx, rx)) = self.serial.release();
        (uart, (tx, rx, self.modem_pins))
    }

    pub fn split(self) -> (Tx<UART, TXPIN, Owned>, Rx<UART, RXPIN, Borrowed>, MODEMPINS) {
        let modem_pins = self.modem_pins;
        let (tx, rx) = self.serial.split();
        (tx, rx, modem_pins)
    }
}

fn new_tx_only<UART, TXPIN>(
    uart: UART,
    pin: TXPIN,
    mut config: Config,
) -> Result<Tx<UART, TXPIN, Owned>, ModeInitError<UART, TXPIN>>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
{
    if let Err(error) = config.validate() {
        return Err(ModeInitError {
            uart,
            pins: pin,
            error: InitErrorKind::InvalidConfig(error),
        });
    }

    let baudrate_divisor = match calc_baudrate_divisor(config.baudrate) {
        Ok(divisor) => divisor,
        Err(error) => {
            return Err(ModeInitError {
                uart,
                pins: pin,
                error: InitErrorKind::InvalidConfig(error),
            });
        }
    };

    config.duplex_mode = DuplexMode::Full;
    config.sync_mode = SyncMode::Async;
    UART::enable_clock();

    if let Err(error) = configure_uart::<UART>(config, baudrate_divisor, true, false) {
        regs::<UART>().control1().modify(|_, w| w.ue().disable());
        return Err(ModeInitError {
            uart,
            pins: pin,
            error,
        });
    }

    Ok(Tx {
        uart: Some(uart),
        pin,
        _uart: PhantomData,
        _owner: PhantomData,
        word_length: config.word_length,
    })
}

fn new_rx_only<UART, RXPIN>(
    uart: UART,
    pin: RXPIN,
    mut config: Config,
) -> Result<Rx<UART, RXPIN, Owned>, ModeInitError<UART, RXPIN>>
where
    UART: Instance,
    RXPIN: RxPin<UART>,
{
    if let Err(error) = config.validate() {
        return Err(ModeInitError {
            uart,
            pins: pin,
            error: InitErrorKind::InvalidConfig(error),
        });
    }

    let baudrate_divisor = match calc_baudrate_divisor(config.baudrate) {
        Ok(divisor) => divisor,
        Err(error) => {
            return Err(ModeInitError {
                uart,
                pins: pin,
                error: InitErrorKind::InvalidConfig(error),
            });
        }
    };

    config.duplex_mode = DuplexMode::Full;
    config.sync_mode = SyncMode::Async;
    UART::enable_clock();

    if let Err(error) = configure_uart::<UART>(config, baudrate_divisor, false, true) {
        regs::<UART>().control1().modify(|_, w| w.ue().disable());
        return Err(ModeInitError {
            uart,
            pins: pin,
            error,
        });
    }

    Ok(Rx {
        uart: Some(uart),
        pin,
        _uart: PhantomData,
        _owner: PhantomData,
        word_length: config.word_length,
    })
}

impl<UART, TXPIN, RXPIN, CLOCKPIN> SynchronousSerial<UART, TXPIN, RXPIN, CLOCKPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
    CLOCKPIN: ClockPin<UART>,
{
    pub fn new(
        uart: UART,
        pins: (TXPIN, RXPIN, CLOCKPIN),
        mut config: Config,
    ) -> Result<Self, ModeInitError<UART, (TXPIN, RXPIN, CLOCKPIN)>> {
        config.duplex_mode = DuplexMode::Full;
        config.sync_mode = SyncMode::Sync;
        let (tx, rx, clock) = pins;

        match Serial::new_configured(uart, (tx, rx), config) {
            Ok(serial) => Ok(Self { serial, clock }),
            Err(error) => {
                let (uart, (tx, rx), error) = error.into_parts();
                Err(ModeInitError {
                    uart,
                    pins: (tx, rx, clock),
                    error,
                })
            }
        }
    }

    pub fn split(self) -> (Tx<UART, TXPIN, Owned>, Rx<UART, RXPIN, Borrowed>, CLOCKPIN) {
        let (tx, rx) = self.serial.split();
        (tx, rx, self.clock)
    }

    pub fn release(self) -> (UART, (TXPIN, RXPIN, CLOCKPIN)) {
        let (uart, (tx, rx)) = self.serial.release();
        (uart, (tx, rx, self.clock))
    }
}

impl<UART, PIN> HalfDuplex<UART, PIN>
where
    UART: Instance,
    PIN: HalfDuplexPin<UART>,
{
    pub fn new(uart: UART, pin: PIN, mut config: Config) -> Result<Self, ModeInitError<UART, PIN>> {
        if let Err(error) = config.validate() {
            return Err(ModeInitError {
                uart,
                pins: pin,
                error: InitErrorKind::InvalidConfig(error),
            });
        }

        let baudrate_divisor = match calc_baudrate_divisor(config.baudrate) {
            Ok(divisor) => divisor,
            Err(error) => {
                return Err(ModeInitError {
                    uart,
                    pins: pin,
                    error: InitErrorKind::InvalidConfig(error),
                });
            }
        };

        config.duplex_mode = DuplexMode::Half;
        config.sync_mode = SyncMode::Async;
        UART::enable_clock();

        if let Err(error) = configure_uart::<UART>(config, baudrate_divisor, true, true) {
            regs::<UART>().control1().modify(|_, w| w.ue().disable());
            return Err(ModeInitError {
                uart,
                pins: pin,
                error,
            });
        }

        Ok(Self {
            uart,
            pin,
            word_length: config.word_length,
        })
    }

    pub fn release(self) -> (UART, PIN) {
        regs::<UART>().control1().modify(|_, w| w.ue().disable());
        (self.uart, self.pin)
    }
}

fn configure_uart<UART: Instance>(
    config: Config,
    baudrate_divisor: u16,
    enable_tx: bool,
    enable_rx: bool,
) -> Result<(), InitErrorKind> {
    let regs = regs::<UART>();

    regs.control1().modify(|_, w| w.ue().disable());

    regs.divider()
        .write(|w| unsafe { w.brr().bits(baudrate_divisor) });

    regs.control1().write(|w| {
        w.idleie()
            .disable()
            .peie()
            .disable()
            .rxneie()
            .disable()
            .tcie()
            .disable()
            .txeie()
            .disable()
    });

    regs.control2().write(|w| {
        w.lbdie()
            .disable()
            .lbm()
            .normal()
            .swap()
            .normal()
            .rxinv()
            .direct()
            .txinv()
            .direct()
            .datainv()
            .direct()
            .msbfirst()
            .lsb()
    });

    regs.control3().write(|w| {
        w.eie()
            .disable()
            .ctsie()
            .disable()
            .ctse()
            .ignored()
            .rtse()
            .bit(false)
    });

    match config.word_length {
        WordLength::DataBits7 => regs.control1().modify(|_, w| w.m()._7bits()),
        WordLength::DataBits8 => regs.control1().modify(|_, w| w.m()._8bits()),
        WordLength::DataBits9 => regs.control1().modify(|_, w| w.m()._9bits()),
    };

    match config.parity {
        Parity::None => regs.control1().modify(|_, w| w.pce().disable()),
        Parity::Even => regs
            .control1()
            .modify(|_, w| w.pce().enable().ps().parity()),
        Parity::Odd => regs.control1().modify(|_, w| w.pce().enable().ps().odd()),
    };

    match config.stop_bits {
        StopBits::Stop1 => regs.control2().modify(|_, w| w.stop_1()._1bit()),
        StopBits::Stop2 => regs.control2().modify(|_, w| w.stop_1()._2bits()),
    };

    match config.duplex_mode {
        DuplexMode::Full => regs.control3().modify(|_, w| w.hdsel().duplex()),
        DuplexMode::Half => regs.control3().modify(|_, w| w.hdsel().half_duplex()),
    };

    match config.sync_mode {
        SyncMode::Async => regs.control2().modify(|_, w| w.clken().asynchronous()),
        SyncMode::Sync => regs.control2().modify(|_, w| w.clken().synchronous()),
    };

    regs.control2().modify(|_, w| {
        w.cpol()
            .bit(config.clock_polarity == ClockPolarity::IdleHigh)
            .cpha()
            .bit(config.clock_phase == ClockPhase::SecondEdge)
            .lbcl()
            .bit(config.clock_last_bit)
            .msbfirst()
            .bit(config.bit_order == BitOrder::MsbFirst)
            .datainv()
            .bit(config.data_inversion)
            .txinv()
            .bit(config.tx_inversion)
            .rxinv()
            .bit(config.rx_inversion)
            .swap()
            .bit(config.swap_pins)
            .lbm()
            .bit(config.loopback)
    });

    regs.control3().modify(|_, w| {
        w.sbkrq()
            .bit(config.tx_break)
            .ovrdis()
            .bit(config.overwrite)
            .rtse()
            .bit(
                matches!(config.flow_control, FlowControl::Rts | FlowControl::RtsCts)
                    && config.rts_mode == RtsMode::Modem,
            )
            .ctse()
            .bit(matches!(
                config.flow_control,
                FlowControl::Cts | FlowControl::RtsCts
            ))
    });

    regs.modem().write(|w| w.dtr().bit(config.modem.dtr));

    match config.dma {
        DmaConfig::None => regs
            .control3()
            .modify(|_, w| w.dmat().disable().dmar().disable()),
        DmaConfig::Tx => regs
            .control3()
            .modify(|_, w| w.dmat().enable().dmar().disable()),
        DmaConfig::Rx => regs
            .control3()
            .modify(|_, w| w.dmat().disable().dmar().enable()),
        DmaConfig::TxRx => regs
            .control3()
            .modify(|_, w| w.dmat().enable().dmar().enable()),
    };

    regs.flags().write(|w| unsafe { w.bits(0x03ff) });

    regs.control1()
        .modify(|_, w| w.te().bit(enable_tx).re().bit(enable_rx).ue().enable());

    for _ in 0..config.init_timeout {
        let flags = regs.flags().read();
        let tx_ready = !enable_tx || flags.teack().bit_is_set();
        let rx_ready = !enable_rx || flags.reack().bit_is_set();
        if tx_ready && rx_ready {
            return Ok(());
        }
        core::hint::spin_loop();
    }

    Err(InitErrorKind::PeripheralNotReady)
}

impl<UART, TXPIN, RXPIN> ErrorType for Serial<UART, TXPIN, RXPIN>
where
    UART: Instance,
    TXPIN: TxPin<UART>,
    RXPIN: RxPin<UART>,
{
    type Error = ErrorKind;
}

impl<UART: Instance, TXPIN, OWNER> ErrorType for Tx<UART, TXPIN, OWNER> {
    type Error = ErrorKind;
}

impl<UART: Instance, TXPIN, OWNER> Tx<UART, TXPIN, OWNER> {
    pub fn send_break(&mut self, enabled: bool) {
        regs::<UART>()
            .control3()
            .modify(|_, w| w.sbkrq().bit(enabled));
    }

    pub fn is_cts_high(&self) -> bool {
        regs::<UART>().flags().read().cts().bit_is_set()
    }

    pub fn cts_changed(&self) -> bool {
        regs::<UART>().flags().read().ctsif().bit_is_set()
    }

    pub fn clear_cts_changed(&mut self) {
        regs::<UART>()
            .flags()
            .write(|w| w.ctsif().clear_bit_by_one());
    }

    /// Starts a non-blocking DMA write using an 8-bit transmit buffer.
    ///
    /// This method consumes the transmitter, DMA channel and buffer and returns
    /// a [`TxDmaTransfer`] guard. The owned parts are returned by
    /// [`TxDmaTransfer::wait`], [`TxDmaTransfer::wait_timeout`] or
    /// [`TxDmaTransfer::abort`]. Dropping the guard aborts the transfer.
    ///
    /// The buffer is accepted through [`embedded_dma::ReadBuffer`]. For a DMA
    /// transfer this usually means using a buffer with a stable address for the
    /// whole transfer, most commonly a `static` buffer or reference. Do not
    /// mutate, move or drop the buffer while the transfer guard is alive.
    ///
    /// DMA completion only means that the last byte has been written into the
    /// USART transmit register. Call [`embedded_hal_nb::serial::Write::flush`]
    /// afterwards if the last bit must have left the pin before continuing.
    ///
    /// Returns [`DmaTransferError::InvalidWordLength`] when the USART is
    /// configured for 9-bit words. Use [`Tx::write_dma_9bit`] in that mode.
    pub fn write_dma<CHANNEL: DmaChannelId, BUFFER>(
        self,
        mut channel: DmaChannel<CHANNEL>,
        buffer: BUFFER,
    ) -> Result<
        TxDmaTransfer<UART, TXPIN, OWNER, CHANNEL, BUFFER>,
        DmaTransferFailure<Self, DmaChannel<CHANNEL>, BUFFER>,
    >
    where
        BUFFER: ReadBuffer<Word = u8>,
    {
        if self.word_length == WordLength::DataBits9 {
            return Err(DmaTransferFailure {
                error: DmaTransferError::InvalidWordLength,
                peripheral: self,
                channel,
                buffer,
            });
        }

        let (source, length) = unsafe { buffer.read_buffer() };
        match dma_write_start::<UART, CHANNEL>(&mut channel, source, length, 0) {
            Ok(request_was_enabled) => Ok(TxDmaTransfer {
                tx: Some(self),
                channel: Some(channel),
                buffer: Some(buffer),
                request_was_enabled,
            }),
            Err(error) => Err(DmaTransferFailure {
                error,
                peripheral: self,
                channel,
                buffer,
            }),
        }
    }

    /// Starts a non-blocking DMA write using a 9-bit transmit buffer.
    ///
    /// This is the 9-bit counterpart of [`Tx::write_dma`]. Each `u16` word must
    /// fit into 9 bits (`0..=0x01ff`); otherwise
    /// [`DmaTransferError::WordOutOfRange`] is returned and all owned parts are
    /// returned in [`DmaTransferFailure`].
    ///
    /// The same lifetime rule applies as for [`Tx::write_dma`]: the DMA buffer
    /// must stay at a stable address and must not be mutated, moved or dropped
    /// while the returned transfer guard is alive.
    pub fn write_dma_9bit<CHANNEL: DmaChannelId, BUFFER>(
        self,
        mut channel: DmaChannel<CHANNEL>,
        buffer: BUFFER,
    ) -> Result<
        TxDmaTransfer<UART, TXPIN, OWNER, CHANNEL, BUFFER>,
        DmaTransferFailure<Self, DmaChannel<CHANNEL>, BUFFER>,
    >
    where
        BUFFER: ReadBuffer<Word = u16>,
    {
        if self.word_length != WordLength::DataBits9 {
            return Err(DmaTransferFailure {
                error: DmaTransferError::InvalidWordLength,
                peripheral: self,
                channel,
                buffer,
            });
        }

        let (source, words) = unsafe { buffer.read_buffer() };
        let values = unsafe { core::slice::from_raw_parts(source, words) };
        if values.iter().any(|word| *word > 0x01ff) {
            return Err(DmaTransferFailure {
                error: DmaTransferError::WordOutOfRange,
                peripheral: self,
                channel,
                buffer,
            });
        }

        match dma_write_start::<UART, CHANNEL>(
            &mut channel,
            source.cast(),
            words.saturating_mul(2),
            1,
        ) {
            Ok(request_was_enabled) => Ok(TxDmaTransfer {
                tx: Some(self),
                channel: Some(channel),
                buffer: Some(buffer),
                request_was_enabled,
            }),
            Err(error) => Err(DmaTransferFailure {
                error,
                peripheral: self,
                channel,
                buffer,
            }),
        }
    }

    /// Performs a blocking DMA write using an 8-bit transmit slice.
    ///
    /// This is a convenience wrapper around the non-blocking DMA machinery. It
    /// borrows the transmitter, channel and slice until the transfer completes
    /// or `timeout` expires.
    ///
    /// DMA completion only means that the last byte has been written into the
    /// USART transmit register. Call [`embedded_hal_nb::serial::Write::flush`]
    /// afterwards if the last bit must have left the pin before continuing.
    pub fn blocking_write_dma<CHANNEL: DmaChannelId>(
        &mut self,
        channel: &mut DmaChannel<CHANNEL>,
        buffer: &[u8],
        timeout: u32,
    ) -> Result<(), DmaTransferError> {
        if self.word_length == WordLength::DataBits9 {
            return Err(DmaTransferError::InvalidWordLength);
        }
        dma_write::<UART, CHANNEL>(channel, buffer.as_ptr(), buffer.len(), 0, timeout)
    }

    /// Performs a blocking DMA write using a 9-bit transmit slice.
    ///
    /// Each `u16` word must fit into 9 bits (`0..=0x01ff`).
    pub fn blocking_write_dma_9bit<CHANNEL: DmaChannelId>(
        &mut self,
        channel: &mut DmaChannel<CHANNEL>,
        buffer: &[u16],
        timeout: u32,
    ) -> Result<(), DmaTransferError> {
        if self.word_length != WordLength::DataBits9 {
            return Err(DmaTransferError::InvalidWordLength);
        }
        if buffer.iter().any(|word| *word > 0x01ff) {
            return Err(DmaTransferError::WordOutOfRange);
        }
        dma_write::<UART, CHANNEL>(
            channel,
            buffer.as_ptr().cast(),
            buffer.len().saturating_mul(2),
            1,
            timeout,
        )
    }
}

impl<UART: Instance, TXPIN> Tx<UART, TXPIN, Owned> {
    pub fn reunite<RXPIN>(self, rx: Rx<UART, RXPIN, Borrowed>) -> Serial<UART, TXPIN, RXPIN>
    where
        TXPIN: TxPin<UART>,
        RXPIN: RxPin<UART>,
    {
        Serial::reunite(self, rx)
    }

    pub fn release(self) -> (UART, TXPIN) {
        regs::<UART>().control1().modify(|_, w| w.ue().disable());
        (
            self.uart.expect("owned USART Tx missing USART owner"),
            self.pin,
        )
    }
}

impl<UART: Instance, RXPIN, OWNER> ErrorType for Rx<UART, RXPIN, OWNER> {
    type Error = ErrorKind;
}

impl<UART: Instance, RXPIN, OWNER> Rx<UART, RXPIN, OWNER> {
    pub fn is_idle(&self) -> bool {
        regs::<UART>().flags().read().idle().bit_is_set()
    }

    pub fn clear_idle(&mut self) {
        regs::<UART>()
            .flags()
            .write(|w| w.idle().clear_bit_by_one());
    }

    pub fn break_detected(&self) -> bool {
        regs::<UART>().flags().read().lbdf().bit_is_set()
    }

    pub fn clear_break_detected(&mut self) {
        regs::<UART>()
            .flags()
            .write(|w| w.lbdf().clear_bit_by_one());
    }

    /// Starts a non-blocking DMA read into an 8-bit receive buffer.
    ///
    /// This method consumes the receiver, DMA channel and buffer and returns an
    /// [`RxDmaTransfer`] guard. The owned parts are returned by
    /// [`RxDmaTransfer::wait`], [`RxDmaTransfer::wait_timeout`] or
    /// [`RxDmaTransfer::abort`]. Dropping the guard aborts the transfer.
    ///
    /// The buffer is accepted through [`embedded_dma::WriteBuffer`]. For a DMA
    /// transfer this usually means using a buffer with a stable address for the
    /// whole transfer, most commonly a `static mut` buffer or another
    /// DMA-safe owner. Do not read, move or drop the buffer while the transfer
    /// guard is alive.
    ///
    /// Returns [`DmaTransferError::InvalidWordLength`] when the USART is
    /// configured for 9-bit words. Use [`Rx::read_dma_9bit`] in that mode.
    pub fn read_dma<CHANNEL: DmaChannelId, BUFFER>(
        self,
        mut channel: DmaChannel<CHANNEL>,
        mut buffer: BUFFER,
    ) -> Result<
        RxDmaTransfer<UART, RXPIN, OWNER, CHANNEL, BUFFER>,
        DmaTransferFailure<Self, DmaChannel<CHANNEL>, BUFFER>,
    >
    where
        BUFFER: WriteBuffer<Word = u8>,
    {
        if self.word_length == WordLength::DataBits9 {
            return Err(DmaTransferFailure {
                error: DmaTransferError::InvalidWordLength,
                peripheral: self,
                channel,
                buffer,
            });
        }

        let (destination, length) = unsafe { buffer.write_buffer() };
        match dma_read_start::<UART, CHANNEL>(&mut channel, destination, length, 0) {
            Ok(request_was_enabled) => Ok(RxDmaTransfer {
                rx: Some(self),
                channel: Some(channel),
                buffer: Some(buffer),
                request_was_enabled,
            }),
            Err(error) => Err(DmaTransferFailure {
                error,
                peripheral: self,
                channel,
                buffer,
            }),
        }
    }

    /// Starts a non-blocking DMA read into a 9-bit receive buffer.
    ///
    /// This is the 9-bit counterpart of [`Rx::read_dma`]. DMA writes one
    /// received word into each `u16` slot.
    ///
    /// The same lifetime rule applies as for [`Rx::read_dma`]: the DMA buffer
    /// must stay at a stable address and must not be read, moved or dropped
    /// while the returned transfer guard is alive.
    pub fn read_dma_9bit<CHANNEL: DmaChannelId, BUFFER>(
        self,
        mut channel: DmaChannel<CHANNEL>,
        mut buffer: BUFFER,
    ) -> Result<
        RxDmaTransfer<UART, RXPIN, OWNER, CHANNEL, BUFFER>,
        DmaTransferFailure<Self, DmaChannel<CHANNEL>, BUFFER>,
    >
    where
        BUFFER: WriteBuffer<Word = u16>,
    {
        if self.word_length != WordLength::DataBits9 {
            return Err(DmaTransferFailure {
                error: DmaTransferError::InvalidWordLength,
                peripheral: self,
                channel,
                buffer,
            });
        }

        let (destination, words) = unsafe { buffer.write_buffer() };
        match dma_read_start::<UART, CHANNEL>(
            &mut channel,
            destination.cast(),
            words.saturating_mul(2),
            1,
        ) {
            Ok(request_was_enabled) => Ok(RxDmaTransfer {
                rx: Some(self),
                channel: Some(channel),
                buffer: Some(buffer),
                request_was_enabled,
            }),
            Err(error) => Err(DmaTransferFailure {
                error,
                peripheral: self,
                channel,
                buffer,
            }),
        }
    }

    /// Performs a blocking DMA read into an 8-bit receive slice.
    ///
    /// This is a convenience wrapper around the non-blocking DMA machinery. It
    /// borrows the receiver, channel and slice until the transfer completes or
    /// `timeout` expires.
    pub fn blocking_read_dma<CHANNEL: DmaChannelId>(
        &mut self,
        channel: &mut DmaChannel<CHANNEL>,
        buffer: &mut [u8],
        timeout: u32,
    ) -> Result<(), DmaTransferError> {
        if self.word_length == WordLength::DataBits9 {
            return Err(DmaTransferError::InvalidWordLength);
        }
        dma_read::<UART, CHANNEL>(channel, buffer.as_mut_ptr(), buffer.len(), 0, timeout)
    }

    /// Performs a blocking DMA read into a 9-bit receive slice.
    ///
    /// DMA writes one received word into each `u16` slot.
    pub fn blocking_read_dma_9bit<CHANNEL: DmaChannelId>(
        &mut self,
        channel: &mut DmaChannel<CHANNEL>,
        buffer: &mut [u16],
        timeout: u32,
    ) -> Result<(), DmaTransferError> {
        if self.word_length != WordLength::DataBits9 {
            return Err(DmaTransferError::InvalidWordLength);
        }
        dma_read::<UART, CHANNEL>(
            channel,
            buffer.as_mut_ptr().cast(),
            buffer.len().saturating_mul(2),
            1,
            timeout,
        )
    }
}

impl<UART: Instance, RXPIN> Rx<UART, RXPIN, Owned> {
    pub fn release(self) -> (UART, RXPIN) {
        regs::<UART>().control1().modify(|_, w| w.ue().disable());
        (
            self.uart.expect("owned USART Rx missing USART owner"),
            self.pin,
        )
    }
}

impl<UART: Instance, TXPIN, OWNER, CHANNEL: DmaChannelId, BUFFER>
    TxDmaTransfer<UART, TXPIN, OWNER, CHANNEL, BUFFER>
{
    pub fn is_done(&mut self) -> Result<bool, DmaTransferError> {
        self.channel
            .as_mut()
            .expect("DMA transfer channel missing")
            .poll()
            .map_err(Into::into)
    }

    pub fn wait(
        mut self,
    ) -> Result<
        (Tx<UART, TXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER),
        DmaTransferFailure<Tx<UART, TXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER>,
    > {
        loop {
            match self.is_done() {
                Ok(true) => return Ok(self.take_parts()),
                Ok(false) => core::hint::spin_loop(),
                Err(error) => return Err(self.take_failure(error)),
            }
        }
    }

    pub fn wait_timeout(
        mut self,
        timeout: u32,
    ) -> Result<
        (Tx<UART, TXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER),
        DmaTransferFailure<Tx<UART, TXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER>,
    > {
        for _ in 0..timeout {
            match self.is_done() {
                Ok(true) => return Ok(self.take_parts()),
                Ok(false) => core::hint::spin_loop(),
                Err(error) => return Err(self.take_failure(error)),
            }
        }
        Err(self.take_failure(DmaTransferError::Dma(DmaError::Timeout)))
    }

    pub fn abort(mut self) -> (Tx<UART, TXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER) {
        self.take_parts()
    }

    fn stop_and_restore(&mut self) {
        if let Some(channel) = self.channel.as_mut() {
            channel.stop();
        }
        regs::<UART>()
            .control3()
            .modify(|_, w| w.dmat().bit(self.request_was_enabled));
    }

    fn take_parts(&mut self) -> (Tx<UART, TXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER) {
        self.stop_and_restore();
        (
            self.tx.take().expect("DMA transfer USART missing"),
            self.channel.take().expect("DMA transfer channel missing"),
            self.buffer.take().expect("DMA transfer buffer missing"),
        )
    }

    fn take_failure(
        &mut self,
        error: DmaTransferError,
    ) -> DmaTransferFailure<Tx<UART, TXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER> {
        let (peripheral, channel, buffer) = self.take_parts();
        DmaTransferFailure {
            error,
            peripheral,
            channel,
            buffer,
        }
    }
}

impl<UART: Instance, TXPIN, OWNER, CHANNEL: DmaChannelId, BUFFER> Drop
    for TxDmaTransfer<UART, TXPIN, OWNER, CHANNEL, BUFFER>
{
    fn drop(&mut self) {
        self.stop_and_restore();
    }
}

impl<UART: Instance, RXPIN, OWNER, CHANNEL: DmaChannelId, BUFFER>
    RxDmaTransfer<UART, RXPIN, OWNER, CHANNEL, BUFFER>
{
    pub fn is_done(&mut self) -> Result<bool, DmaTransferError> {
        self.channel
            .as_mut()
            .expect("DMA transfer channel missing")
            .poll()
            .map_err(Into::into)
    }

    pub fn wait(
        mut self,
    ) -> Result<
        (Rx<UART, RXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER),
        DmaTransferFailure<Rx<UART, RXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER>,
    > {
        loop {
            match self.is_done() {
                Ok(true) => return Ok(self.take_parts()),
                Ok(false) => core::hint::spin_loop(),
                Err(error) => return Err(self.take_failure(error)),
            }
        }
    }

    pub fn wait_timeout(
        mut self,
        timeout: u32,
    ) -> Result<
        (Rx<UART, RXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER),
        DmaTransferFailure<Rx<UART, RXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER>,
    > {
        for _ in 0..timeout {
            match self.is_done() {
                Ok(true) => return Ok(self.take_parts()),
                Ok(false) => core::hint::spin_loop(),
                Err(error) => return Err(self.take_failure(error)),
            }
        }
        Err(self.take_failure(DmaTransferError::Dma(DmaError::Timeout)))
    }

    pub fn abort(mut self) -> (Rx<UART, RXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER) {
        self.take_parts()
    }

    fn stop_and_restore(&mut self) {
        if let Some(channel) = self.channel.as_mut() {
            channel.stop();
        }
        regs::<UART>()
            .control3()
            .modify(|_, w| w.dmar().bit(self.request_was_enabled));
    }

    fn take_parts(&mut self) -> (Rx<UART, RXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER) {
        self.stop_and_restore();
        (
            self.rx.take().expect("DMA transfer USART missing"),
            self.channel.take().expect("DMA transfer channel missing"),
            self.buffer.take().expect("DMA transfer buffer missing"),
        )
    }

    fn take_failure(
        &mut self,
        error: DmaTransferError,
    ) -> DmaTransferFailure<Rx<UART, RXPIN, OWNER>, DmaChannel<CHANNEL>, BUFFER> {
        let (peripheral, channel, buffer) = self.take_parts();
        DmaTransferFailure {
            error,
            peripheral,
            channel,
            buffer,
        }
    }
}

impl<UART: Instance, RXPIN, OWNER, CHANNEL: DmaChannelId, BUFFER> Drop
    for RxDmaTransfer<UART, RXPIN, OWNER, CHANNEL, BUFFER>
{
    fn drop(&mut self) {
        self.stop_and_restore();
    }
}

impl<UART, PIN> ErrorType for HalfDuplex<UART, PIN>
where
    UART: Instance,
    PIN: HalfDuplexPin<UART>,
{
    type Error = ErrorKind;
}

impl<UART, PIN> Write for HalfDuplex<UART, PIN>
where
    UART: Instance,
    PIN: HalfDuplexPin<UART>,
{
    fn write(&mut self, byte: u8) -> NbResult<(), Self::Error> {
        if self.word_length == WordLength::DataBits9 {
            return Err(NbError::Other(ErrorKind::Other));
        }
        write_word::<UART>(u16::from(byte))
    }

    fn flush(&mut self) -> NbResult<(), Self::Error> {
        flush::<UART>()
    }
}

impl<UART, PIN> Write<u16> for HalfDuplex<UART, PIN>
where
    UART: Instance,
    PIN: HalfDuplexPin<UART>,
{
    fn write(&mut self, word: u16) -> NbResult<(), Self::Error> {
        if self.word_length != WordLength::DataBits9 || word > 0x01ff {
            return Err(NbError::Other(ErrorKind::Other));
        }
        write_word::<UART>(word)
    }

    fn flush(&mut self) -> NbResult<(), Self::Error> {
        flush::<UART>()
    }
}

impl<UART, PIN> Read for HalfDuplex<UART, PIN>
where
    UART: Instance,
    PIN: HalfDuplexPin<UART>,
{
    fn read(&mut self) -> NbResult<u8, Self::Error> {
        if self.word_length == WordLength::DataBits9 {
            return Err(NbError::Other(ErrorKind::Other));
        }
        read_word::<UART>().map(|word| word as u8)
    }
}

impl<UART, PIN> Read<u16> for HalfDuplex<UART, PIN>
where
    UART: Instance,
    PIN: HalfDuplexPin<UART>,
{
    fn read(&mut self) -> NbResult<u16, Self::Error> {
        if self.word_length != WordLength::DataBits9 {
            return Err(NbError::Other(ErrorKind::Other));
        }
        read_word::<UART>()
    }
}

impl<UART: Instance, TXPIN, OWNER> Write for Tx<UART, TXPIN, OWNER> {
    fn write(&mut self, byte: u8) -> NbResult<(), Self::Error> {
        if self.word_length == WordLength::DataBits9 {
            return Err(NbError::Other(ErrorKind::Other));
        }
        write_word::<UART>(u16::from(byte))
    }

    fn flush(&mut self) -> NbResult<(), Self::Error> {
        flush::<UART>()
    }
}

impl<UART: Instance, TXPIN, OWNER> Write<u16> for Tx<UART, TXPIN, OWNER> {
    fn write(&mut self, word: u16) -> NbResult<(), Self::Error> {
        if self.word_length != WordLength::DataBits9 || word > 0x01ff {
            return Err(NbError::Other(ErrorKind::Other));
        }
        write_word::<UART>(word)
    }

    fn flush(&mut self) -> NbResult<(), Self::Error> {
        flush::<UART>()
    }
}

impl<UART: Instance, TXPIN, OWNER> fmt::Write for Tx<UART, TXPIN, OWNER> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for byte in s.bytes() {
            nb_block(|| self.write(byte)).map_err(|_| fmt::Error)?;
        }

        nb_block(|| <Self as Write<u8>>::flush(self)).map_err(|_| fmt::Error)
    }
}

impl<UART: Instance, RXPIN, OWNER> Read for Rx<UART, RXPIN, OWNER> {
    fn read(&mut self) -> NbResult<u8, Self::Error> {
        if self.word_length == WordLength::DataBits9 {
            return Err(NbError::Other(ErrorKind::Other));
        }
        read_word::<UART>().map(|word| word as u8)
    }
}

impl<UART: Instance, RXPIN, OWNER> Read<u16> for Rx<UART, RXPIN, OWNER> {
    fn read(&mut self) -> NbResult<u16, Self::Error> {
        if self.word_length != WordLength::DataBits9 {
            return Err(NbError::Other(ErrorKind::Other));
        }
        read_word::<UART>()
    }
}

fn dma_write_start<UART: Instance, CHANNEL: DmaChannelId>(
    channel: &mut DmaChannel<CHANNEL>,
    source: *const u8,
    length: usize,
    size: u32,
) -> Result<bool, DmaTransferError> {
    let regs = regs::<UART>();
    let destination = core::ptr::from_ref(regs.txdata()).cast_mut().cast::<u8>();
    let request_was_enabled = regs.control3().read().dmat().bit_is_set();
    regs.control3().modify(|_, w| w.dmat().enable());

    if let Err(error) = channel.start(source, destination, length, dma_tx_config::<UART>(size)) {
        regs.control3()
            .modify(|_, w| w.dmat().bit(request_was_enabled));
        return Err(error.into());
    }
    Ok(request_was_enabled)
}

fn dma_read_start<UART: Instance, CHANNEL: DmaChannelId>(
    channel: &mut DmaChannel<CHANNEL>,
    destination: *mut u8,
    length: usize,
    size: u32,
) -> Result<bool, DmaTransferError> {
    let regs = regs::<UART>();
    let source = core::ptr::from_ref(regs.rxdata()).cast::<u8>();
    let request_was_enabled = regs.control3().read().dmar().bit_is_set();
    regs.control3().modify(|_, w| w.dmar().enable());

    if let Err(error) = channel.start(source, destination, length, dma_rx_config::<UART>(size)) {
        regs.control3()
            .modify(|_, w| w.dmar().bit(request_was_enabled));
        return Err(error.into());
    }
    Ok(request_was_enabled)
}

fn dma_tx_config<UART: Instance>(size: u32) -> u32 {
    (1 << 3) | (1 << 5) | (size << 7) | (size << 9) | (UART::DMA_REQUEST << 21) | (1 << 26)
}

fn dma_rx_config<UART: Instance>(size: u32) -> u32 {
    (1 << 4) | (1 << 6) | (size << 7) | (size << 9) | (UART::DMA_REQUEST << 17) | (1 << 25)
}

fn dma_write<UART: Instance, CHANNEL: DmaChannelId>(
    channel: &mut DmaChannel<CHANNEL>,
    source: *const u8,
    length: usize,
    size: u32,
    timeout: u32,
) -> Result<(), DmaTransferError> {
    let regs = regs::<UART>();
    let destination = core::ptr::from_ref(regs.txdata()).cast_mut().cast::<u8>();
    let config = dma_tx_config::<UART>(size);

    let request_was_enabled = regs.control3().read().dmat().bit_is_set();
    regs.control3().modify(|_, w| w.dmat().enable());
    let result = channel.transfer(source, destination, length, config, timeout);
    regs.control3()
        .modify(|_, w| w.dmat().bit(request_was_enabled));
    result.map_err(Into::into)
}

fn dma_read<UART: Instance, CHANNEL: DmaChannelId>(
    channel: &mut DmaChannel<CHANNEL>,
    destination: *mut u8,
    length: usize,
    size: u32,
    timeout: u32,
) -> Result<(), DmaTransferError> {
    let regs = regs::<UART>();
    let source = core::ptr::from_ref(regs.rxdata()).cast::<u8>();
    let config = dma_rx_config::<UART>(size);

    let request_was_enabled = regs.control3().read().dmar().bit_is_set();
    regs.control3().modify(|_, w| w.dmar().enable());
    let result = channel.transfer(source, destination, length, config, timeout);
    regs.control3()
        .modify(|_, w| w.dmar().bit(request_was_enabled));
    result.map_err(Into::into)
}

fn write_word<UART: Instance>(word: u16) -> NbResult<(), ErrorKind> {
    let regs = regs::<UART>();
    if regs.flags().read().txe().bit_is_clear() {
        return Err(NbError::WouldBlock);
    }
    regs.txdata().write(|w| unsafe { w.tdr().bits(word) });
    Ok(())
}

fn flush<UART: Instance>() -> NbResult<(), ErrorKind> {
    if regs::<UART>().flags().read().tc().bit_is_clear() {
        return Err(NbError::WouldBlock);
    }
    Ok(())
}

fn read_word<UART: Instance>() -> NbResult<u16, ErrorKind> {
    let regs = regs::<UART>();
    let flags = regs.flags().read();

    if flags.pe().bit_is_set() {
        regs.flags().write(|w| w.pe().clear_bit_by_one());
        return Err(NbError::Other(ErrorKind::Parity));
    }
    if flags.fe().bit_is_set() {
        regs.flags().write(|w| w.fe().clear_bit_by_one());
        return Err(NbError::Other(ErrorKind::FrameFormat));
    }
    if flags.nf().bit_is_set() {
        regs.flags().write(|w| w.nf().clear_bit_by_one());
        return Err(NbError::Other(ErrorKind::Noise));
    }
    if flags.ore().bit_is_set() {
        regs.flags().write(|w| w.ore().clear_bit_by_one());
        return Err(NbError::Other(ErrorKind::Overrun));
    }
    if flags.rxne().bit_is_clear() {
        return Err(NbError::WouldBlock);
    }

    Ok(regs.rxdata().read().rdr().bits())
}

#[inline(always)]
fn regs<UART: Instance>() -> &'static RegisterBlock {
    unsafe { &*UART::ptr() }
}

#[inline(always)]
fn calc_baudrate_divisor(baudrate: u32) -> Result<u16, ConfigError> {
    if baudrate == 0 {
        return Err(ConfigError::ZeroBaudrate);
    }

    let sys_clock: u32 = system_clock().into();
    let p = unsafe { Peripherals::steal() };
    let ahb_divisor = p.pm.div_ahb().read().bits().saturating_add(1);
    let apb_p_divisor = p.pm.div_apb_p().read().bits().saturating_add(1);
    let clock = sys_clock / ahb_divisor / apb_p_divisor;
    let divisor = (u64::from(clock) + u64::from(baudrate) / 2) / u64::from(baudrate);

    if divisor < 16 {
        return Err(ConfigError::BaudrateTooHigh);
    }
    if divisor > u64::from(u16::MAX) {
        return Err(ConfigError::BaudrateTooLow);
    }

    Ok(divisor as u16)
}

fn nb_block<T, E>(mut f: impl FnMut() -> NbResult<T, E>) -> Result<T, E> {
    loop {
        match f() {
            Ok(value) => return Ok(value),
            Err(NbError::WouldBlock) => core::hint::spin_loop(),
            Err(NbError::Other(error)) => return Err(error),
        }
    }
}
