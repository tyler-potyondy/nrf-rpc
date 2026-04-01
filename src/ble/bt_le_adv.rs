use core::ptr;

// Assuming these types are defined elsewhere in your Rust Bluetooth bindings
type BtAddrLe = *const u8; // Placeholder for bt_addr_le_t

// Constants (you'll need to define these based on the C values)
const BT_ID_DEFAULT: u8 = 0;
const BT_LE_ADV_OPT_CONN: u32 = 0x03; // BIT(0) | BIT(1)
const BT_LE_ADV_OPT_DIR_MODE_LOW_DUTY: u32 = 0x10; // BIT(4)
const BT_LE_ADV_OPT_USE_IDENTITY: u32 = 0x04; // BIT(2)
const BT_LE_ADV_OPT_EXT_ADV: u32 = 0x400; // BIT(10)
const BT_LE_ADV_OPT_SCANNABLE: u32 = 0x200; // BIT(9)
const BT_LE_ADV_OPT_CODED: u32 = 0x1000; // BIT(12)
const BT_GAP_ADV_FAST_INT_MIN_2: u32 = 0x00A0; // 100ms
const BT_GAP_ADV_FAST_INT_MAX_2: u32 = 0x00F0; // 150ms

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BtLeAdvParam {
    pub id: u8,
    pub sid: u8,
    pub secondary_max_skip: u8,
    pub options: u32,
    pub interval_min: u32,
    pub interval_max: u32,
    pub peer: BtAddrLe,
}

impl BtLeAdvParam {
    const fn new(options: u32, interval_min: u32, interval_max: u32, peer: BtAddrLe) -> Self {
        Self {
            id: BT_ID_DEFAULT,
            sid: 0,
            secondary_max_skip: 0,
            options,
            interval_min,
            interval_max,
            peer,
        }
    }
}

/// GAP recommended connectable advertising parameters for non-connectable advertising events
pub const BT_LE_ADV_CONN_FAST_2: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_CONN,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Low duty cycle directed advertising
pub const fn bt_le_adv_conn_dir_low_duty(peer: BtAddrLe) -> BtLeAdvParam {
    BtLeAdvParam::new(
        BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_DIR_MODE_LOW_DUTY,
        BT_GAP_ADV_FAST_INT_MIN_2,
        BT_GAP_ADV_FAST_INT_MAX_2,
        peer,
    )
}

/// Non-connectable advertising with private address
pub const BT_LE_ADV_NCONN: BtLeAdvParam = BtLeAdvParam::new(
    0,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Non-connectable advertising with identity
pub const BT_LE_ADV_NCONN_IDENTITY: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_USE_IDENTITY,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Connectable extended advertising
pub const BT_LE_EXT_ADV_CONN: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Scannable extended advertising
pub const BT_LE_EXT_ADV_SCAN: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_SCANNABLE,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Non-connectable extended advertising with private address
pub const BT_LE_EXT_ADV_NCONN: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_EXT_ADV,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Non-connectable extended advertising with identity
pub const BT_LE_EXT_ADV_NCONN_IDENTITY: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Non-connectable extended advertising on coded PHY with private address
pub const BT_LE_EXT_ADV_CODED_NCONN: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);

/// Non-connectable extended advertising on coded PHY with identity
pub const BT_LE_EXT_ADV_CODED_NCONN_IDENTITY: BtLeAdvParam = BtLeAdvParam::new(
    BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED | BT_LE_ADV_OPT_USE_IDENTITY,
    BT_GAP_ADV_FAST_INT_MIN_2,
    BT_GAP_ADV_FAST_INT_MAX_2,
    ptr::null(),
);
