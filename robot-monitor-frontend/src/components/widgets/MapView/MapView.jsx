import { useEffect, useRef, useState } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
// import { MapContainer, TileLayer, Marker, useMap } from "react-leaflet";

import { MAP_CONFIG } from "../../../config/constants";
import "./MapView.css";

function MapResizeHandler() {
  const map = useMap();
  const containerRef = useRef(null);

  useEffect(() => {
    // Observe container size changes
    const resizeObserver = new ResizeObserver(() => {
      map.invalidateSize();
    });

    const container = map.getContainer();
    resizeObserver.observe(container);

    return () => {
      resizeObserver.disconnect();
    };
  }, [map]);

  return null;
}

export function MapView({ gps, path, odom }) {
  const mapRef = useRef(null);
  const mapInstanceRef = useRef(null);
  const markerRef = useRef(null);
  const pathLayerRef = useRef(null);
  const [isFollowing, setIsFollowing] = useState(true);
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

    map.on("dragstart", () => {
      setIsFollowing(false);
    });
    mapInstanceRef.current = map;

    const resizeObserver = new ResizeObserver(() => {
      map.invalidateSize(); // update map size on container resize

      //
      if (isFollowing && gps) {
        map.setView([gps.latitude, gps.longitude], map.getZoom());
      }
    });

    resizeObserver.observe(mapRef.current);
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
    // mapInstanceRef.current.setView(
    //   [latitude, longitude],
    //   mapInstanceRef.current.getZoom()
    // );

    if (isFollowing) {
      mapInstanceRef.current.setView(
        [latitude, longitude],
        mapInstanceRef.current.getZoom(),
        { animate: true, duration: 0.5 }
      );
    }
  }, [gps, odom, isFollowing]);

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

  const centerOnRobot = () => {
    if (mapInstanceRef.current && gps) {
      setIsFollowing(true);
      mapInstanceRef.current.setView(
        [gps.latitude, gps.longitude],
        mapInstanceRef.current.getZoom(),
        { animate: true, duration: 0.8 }
      );
    }
  };
  return (
    <div className="map-widget  map-interactive">
      <div className="map-widget__header widget-header">
        <h3 className="map-widget__title">Map</h3>
      </div>
      <div ref={mapRef} className="map-widget__map" />
      {!isFollowing && gps && (
        <button
          onClick={centerOnRobot}
          className="map-center-btn-glass"
          title="Center on robot"
        >
          <span className="map-center-btn-glass__icon">🎯</span>
        </button>
      )}

      {isFollowing && <div className="map-following-badge">🔄 Following</div>}
    </div>
  );
}
