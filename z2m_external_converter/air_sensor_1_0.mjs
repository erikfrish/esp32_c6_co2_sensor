import {co2, humidity, pressure, temperature} from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    fingerprint: [
        {
            modelID: 'Air Sensor 1.0',
            manufacturerName: 'erikfrish',
        },
    ],
    model: 'ESP32-C6 Air Sensor',
    vendor: 'erikfrish',
    description: 'Air quality sensor with CO2, temperature, humidity, and pressure',
    extend: [
        temperature(),
        humidity(),
        pressure(),
        co2(),
    ],
};
