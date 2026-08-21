// witmotion.js
//
// Minimal Web Bluetooth client for WitMotion BLE IMU sensors (the WT901BLE / BWT901BLE / *BLECL
// family, which all share the same GATT layout and packet format). Connects to the sensor's
// standard service, subscribes to notifications, and parses the WitMotion binary packet format
// into readable samples of the form { ax, ay, az, gx, gy, gz, roll, pitch, yaw, t }.
//
// Units: acceleration in g (multiples of standard gravity), gyro in degrees/second, angles in
// degrees, t in milliseconds (performance.now()).
//
// Protocol references:
//   https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol
//   https://wit-motion.gitbook.io/witmotion-sdk/ble-5.0-protocol
//
// If your specific module advertises different GATT UUIDs, pass overrides to the constructor:
//   new WitMotionSensor({ serviceUuid: '...', notifyCharUuid: '...' })

const DEFAULT_SERVICE_UUID = '0000ffe5-0000-1000-8000-00805f9a34fb';
const DEFAULT_NOTIFY_CHAR_UUID = '0000ffe4-0000-1000-8000-00805f9a34fb';
// Write characteristic — only needed if you later want to send config commands (e.g. change the
// output rate). Not required just to read data, so it's optional and unused by default.
const DEFAULT_WRITE_CHAR_UUID = '0000ffe9-0000-1000-8000-00805f9a34fb';

function toSignedInt16LE(lo, hi) {
  const v = (hi << 8) | lo;
  return v & 0x8000 ? v - 0x10000 : v;
}

export class WitMotionSensor extends EventTarget {
  constructor(options = {}) {
    super();
    this.serviceUuid = options.serviceUuid || DEFAULT_SERVICE_UUID;
    this.notifyCharUuid = options.notifyCharUuid || DEFAULT_NOTIFY_CHAR_UUID;
    this.writeCharUuid = options.writeCharUuid || DEFAULT_WRITE_CHAR_UUID;

    this.device = null;
    this.server = null;
    this.notifyChar = null;
    this.writeChar = null;
    this._byteBuffer = [];
  }

  get connected() {
    return !!(this.device && this.device.gatt && this.device.gatt.connected);
  }

  // Prompts the browser's device picker (must be called from a user gesture, e.g. a click
  // handler), connects, and starts streaming readings as 'reading' events.
  async connect() {
    if (!('bluetooth' in navigator)) {
      throw new Error('Web Bluetooth is not available in this browser. Use Chrome or Edge over HTTPS (or http://localhost).');
    }

    this.device = await navigator.bluetooth.requestDevice({
      filters: [{ services: [this.serviceUuid] }],
      optionalServices: [this.serviceUuid],
    });

    this.device.addEventListener('gattserverdisconnected', () => {
      this.dispatchEvent(new CustomEvent('disconnected'));
    });

    this.server = await this.device.gatt.connect();
    const service = await this.server.getPrimaryService(this.serviceUuid);
    this.notifyChar = await service.getCharacteristic(this.notifyCharUuid);

    try {
      this.writeChar = await service.getCharacteristic(this.writeCharUuid);
    } catch (err) {
      this.writeChar = null; // optional — fine if this module doesn't expose a write characteristic
    }

    this.notifyChar.addEventListener('characteristicvaluechanged', (event) => {
      this._handleNotification(event.target.value);
    });
    await this.notifyChar.startNotifications();

    this.dispatchEvent(new CustomEvent('connected', { detail: { name: this.device.name } }));
    return this.device;
  }

  async disconnect() {
    if (this.device && this.device.gatt && this.device.gatt.connected) {
      this.device.gatt.disconnect();
    }
  }

  _handleNotification(dataView) {
    for (let i = 0; i < dataView.byteLength; i++) {
      this._byteBuffer.push(dataView.getUint8(i));
    }
    this._drainBuffer();
  }

  // WitMotion packets are 0x55-prefixed, self-checksummed frames. BLE notifications don't always
  // line up one-to-one with packets, so bytes are buffered and scanned for valid frames; anything
  // that doesn't check out (misaligned start, torn packet) is resynced byte-by-byte.
  _drainBuffer() {
    const buf = this._byteBuffer;
    while (buf.length > 0) {
      if (buf[0] !== 0x55) {
        buf.shift();
        continue;
      }
      if (buf.length < 2) return; // wait for more data

      const type = buf[1];
      const packetLen = type === 0x61 ? 20 : 11; // 0x61 = combined accel+gyro+angle packet
      if (buf.length < packetLen) return; // wait for the rest of the packet

      const packet = buf.slice(0, packetLen);
      const sum = packet.slice(0, packetLen - 1).reduce((a, b) => (a + b) & 0xff, 0);
      if (sum !== packet[packetLen - 1]) {
        // Bad checksum — we were misaligned. Drop just the header byte and try to resync.
        buf.shift();
        continue;
      }

      buf.splice(0, packetLen);
      this._parsePacket(packet);
    }
  }

  _parsePacket(bytes) {
    const type = bytes[1];
    const body = bytes.slice(2, bytes.length - 1); // strip header/type byte and trailing checksum
    const shorts = [];
    for (let i = 0; i + 1 < body.length; i += 2) {
      shorts.push(toSignedInt16LE(body[i], body[i + 1]));
    }

    const reading = { t: performance.now() };

    if (type === 0x61 && shorts.length >= 9) {
      // Combined packet used by most current WitMotion BLE modules (BLE 5.0 / *BLECL): accel,
      // gyro, and angle in one 20-byte frame.
      const [ax, ay, az, gx, gy, gz, roll, pitch, yaw] = shorts;
      reading.ax = (ax / 32768) * 16;
      reading.ay = (ay / 32768) * 16;
      reading.az = (az / 32768) * 16;
      reading.gx = (gx / 32768) * 2000;
      reading.gy = (gy / 32768) * 2000;
      reading.gz = (gz / 32768) * 2000;
      reading.roll = (roll / 32768) * 180;
      reading.pitch = (pitch / 32768) * 180;
      reading.yaw = (yaw / 32768) * 180;
      this.dispatchEvent(new CustomEvent('reading', { detail: reading }));
    } else if (type === 0x51 && shorts.length >= 3) {
      // Older/standard-protocol modules send acceleration as its own 11-byte packet instead.
      const [ax, ay, az] = shorts;
      reading.ax = (ax / 32768) * 16;
      reading.ay = (ay / 32768) * 16;
      reading.az = (az / 32768) * 16;
      this.dispatchEvent(new CustomEvent('reading', { detail: reading }));
    }
    // Other packet types (time 0x50, gyro-only 0x52, angle-only 0x53, magnetometer 0x54, ...) are
    // ignored here since stroke detection only needs acceleration.
  }
}
