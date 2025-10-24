import { useEffect, useRef } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { MAP_CONFIG } from "../../../config/constants";
import "./MapView.css";

export function MapView({ gps, path, odom }) {
  const mapRef = useRef(null);
  const mapInstanceRef = useRef(null);
  const markerRef = useRef(null);
  const pathLayerRef = useRef(null);

  // init map
  useEffect(() => {
    if (!mapRef.current || mapInstanceRef.current) return;

    const map = L.map(mapRef.current, {
      center: MAP_CONFIG.defaultCenter,
      zoom: MAP_CONFIG.defaultZoom,
      zoomControl: true,
    });

    // (OpenStreetMap)
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: "© OpenStreetMap contributors",
      maxZoom: MAP_CONFIG.maxZoom,
      minZoom: MAP_CONFIG.minZoom,
    }).addTo(map);

    mapInstanceRef.current = map;

    return () => {
      if (mapInstanceRef.current) {
        mapInstanceRef.current.remove();
        mapInstanceRef.current = null;
      }
    };
  }, []);

  // update robot marker
  useEffect(() => {
    if (!mapInstanceRef.current || !gps) return;

    const { latitude, longitude } = gps;

    if (!markerRef.current) {
      // custom icon for robot
      const robotIcon = L.divIcon({
        className: "robot-marker",
        html: '<div class="robot-marker__dot"></div>',
        iconSize: [20, 20],
        iconAnchor: [10, 10],
      });

      markerRef.current = L.marker([latitude, longitude], {
        icon: robotIcon,
      }).addTo(mapInstanceRef.current);
    } else {
      // update position
      markerRef.current.setLatLng([latitude, longitude]);
    }

    // center map on robot
    mapInstanceRef.current.setView(
      [latitude, longitude],
      mapInstanceRef.current.getZoom()
    );
  }, [gps]);

  useEffect(() => {
    if (!mapInstanceRef.current || !gps) return;

    const { latitude, longitude } = gps;

    let rotation = 0;
    if (odom?.orientation_z && odom?.orientation_w) {
      const heading = Math.atan2(
        2.0 * (odom.orientation_w * odom.orientation_z),
        1.0 - 2.0 * (odom.orientation_z * odom.orientation_z)
      );
      rotation = (heading * 180) / Math.PI - 90;
    }

    if (!markerRef.current) {
      const robotIcon = L.divIcon({
        className: "robot-marker",
        html: `<div class="robot-marker__arrow" style="transform: rotate(${rotation}deg)">
                 <svg width="24" height="24" viewBox="0 0 24 24">
                   <path d="M12 2 L20 20 L12 16 L4 20 Z" fill="#FF4444" stroke="#FFF" stroke-width="1.5"/>
                 </svg>
               </div>`,
        iconSize: [24, 24],
        iconAnchor: [12, 12],
      });

      markerRef.current = L.marker([latitude, longitude], {
        icon: robotIcon,
      }).addTo(mapInstanceRef.current);
    } else {
      markerRef.current.setLatLng([latitude, longitude]);

      const iconElement = markerRef.current.getElement();
      if (iconElement) {
        const arrow = iconElement.querySelector(".robot-marker__arrow");
        if (arrow) {
          arrow.style.transform = `rotate(${rotation}deg)`;
        }
      }
    }

    mapInstanceRef.current.setView(
      [latitude, longitude],
      mapInstanceRef.current.getZoom()
    );
  }, [gps, odom]);

  // update the path
  useEffect(() => {
    if (!mapInstanceRef.current || !path || path.length < 2) return;

    if (pathLayerRef.current) {
      // remove old path
      mapInstanceRef.current.removeLayer(pathLayerRef.current);
    }

    // draw new
    const pathCoords = path.map((p) => [p.lat, p.lng]);
    pathLayerRef.current = L.polyline(pathCoords, {
      color: MAP_CONFIG.pathColor,
      weight: MAP_CONFIG.pathWeight,
      opacity: 0.7,
    }).addTo(mapInstanceRef.current);
  }, [path]);

  return (
    <div className="map-widget  map-interactive">
      <div className="map-widget__header widget-header">
        <h3 className="map-widget__title">Map</h3>
      </div>
      <div ref={mapRef} className="map-widget__map" />
    </div>
  );
}
