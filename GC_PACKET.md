# `GC` (servo control)

Purpose: command the GNSS tracking yagi antenna servo controller from `/antenna`.

Route: frontend `/antenna` -> Mac `dashboard_bridge` WebSocket serial role `servo` / `SERVO (GC)` -> servo controller serial port.

Wire:

| Item | Value |
|---|---|
| Header | `b"GC"` |
| `struct` | `<2sHBhH` |
| Size | 9 bytes |
| CRC | CRC-16/CCITT-FALSE over first 7 bytes |

Fields:

| Byte | Field | Type | Notes |
|---:|---|---|---|
| 0..1 | header | `char[2]` | ASCII `GC` |
| 2..3 | seq | `uint16` | sender sequence, little-endian |
| 4 | type | `uint8` | see type table below |
| 5..6 | value | `int16` | type-specific data, little-endian |
| 7..8 | crc16 | `uint16` | little-endian CRC |

Types:

| Type | Name | `value` semantics |
|---:|---|---|
| `0x01` | `AUTO` | `0..3599`, antenna tracking angle in `0.1 deg` units; values above `1800` are applied as `1800` |
| `0x02` | `MANUAL_POSITION` | `0..3599`, absolute angle in `0.1 deg` units; values above `1800` are applied as `1800` |
| `0x03` | `MANUAL_RATE` | `-1000..1000`, rate command where `1000 = +100.0%`; the controller slews the commanded target angle in software |
| `0x04` | `STOP` | `0` |
| `0x05` | `HOME` | `0` |

Servo semantics:

- `AUTO` and `MANUAL_POSITION` carry absolute position commands, not relative deltas.
- `/antenna` may compute a 270-degree servo absolute angle for GNSS tracking and send it as `AUTO`; this controller limits the operational range to `0..1800`.
- The servo mechanical angle is not a map bearing. Map bearing is only an internal calculation input for `/antenna`.
