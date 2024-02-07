import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, useMap } from 'react-leaflet';
import sensorConfig from '../sensorConfig.json';
import 'leaflet/dist/leaflet.css';

const UpdateMapView = ({ position }) => {
    const map = useMap();
    useEffect(() => {
      if (position) {
        map.setView(position, map.getZoom());
      }
    }, [position, map]);

    return null;
};

const MyMap = () => {
  const [position, setPosition] = useState(null);
  

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:9090');
    let dataTimeout;

    ws.onopen = () => {
      // ROSbridge를 통해 NavSatFix 메시지 구독
      const gpsConfig = sensorConfig.GPS;
      ws.send(JSON.stringify({
        op: 'subscribe',
        topic: gpsConfig.topic, // GPS topic from sensorConfig
        type: gpsConfig.messageType // GPS message type from sensorConfig
      }));
    };

    ws.onmessage = (event) => {
      const message = JSON.parse(event.data);
      if (message.msg) {
        const { latitude, longitude } = message.msg;
        setPosition([latitude, longitude]);

        // Reset the timer on new data
        clearTimeout(dataTimeout);
        dataTimeout = setTimeout(() => {
          setPosition(null);
        }, 3000); // Set to null after 5 seconds of no new data
      }
    };

    return () => {
      ws.close();
      clearTimeout(dataTimeout);
    };
  }, []);

  return (
    <MapContainer center={[37.38402987655305, 126.65707983143997]} zoom={16} style={{ height: "100%", width: "100%" }}>
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      />
      {position && <Marker position={position}></Marker>}
      {position && <UpdateMapView position={position} />}
    </MapContainer>
  );
};

export default MyMap;