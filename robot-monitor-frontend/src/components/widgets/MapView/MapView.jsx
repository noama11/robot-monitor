import { useEffect, useRef } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { MAP_CONFIG } from "../../../config/constants";
import "./MapView.css";

export function MapView({ gps, path }) {
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
    <div className="map-widget">
      <div className="map-widget__header widget-header">
        <h3 className="map-widget__title">Map</h3>
      </div>
      <div ref={mapRef} className="map-widget__map" />
    </div>
  );
}
