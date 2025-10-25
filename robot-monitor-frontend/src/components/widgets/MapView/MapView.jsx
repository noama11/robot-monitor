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
  const visionConeRef = useRef(null);
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
      const map = mapInstanceRef.current;
      if (map && map._container) {
        map.invalidateSize(); // update map size on container resize
      }

      //
      if (isFollowing && gps) {
        map.setView([gps.latitude, gps.longitude], map.getZoom());
      }
    });

    resizeObserver.observe(mapRef.current);
    return () => {
      resizeObserver.disconnect(); // cleanup observer
      if (mapInstanceRef.current) {
        mapInstanceRef.current.remove();
        mapInstanceRef.current = null;
      }
    };
  }, []);
  const toRad = (deg) => (deg * Math.PI) / 180;
  const createVisionCone = (
    lat,
    lng,
    headingDeg,
    radiusDeg = 0.0003,
    coneAngleDeg = 45
  ) => {
    const numPoints = 24;
    const points = [];
    const center = [lat, lng];
    points.push(center);

    const headingRad = toRad(headingDeg);
    const halfConeRad = toRad(coneAngleDeg);

    const cosLat = Math.cos(toRad(lat)) || 1;

    for (let i = 0; i <= numPoints; i++) {
      const rel = -halfConeRad + (2 * halfConeRad * i) / numPoints;
      const a = headingRad + rel;

      const dLat = radiusDeg * Math.cos(a);
      const dLng = (radiusDeg * Math.sin(a)) / cosLat;

      points.push([lat + dLat, lng + dLng]);
    }

    points.push(center);
    return points;
  };

  // update robot marker
  useEffect(() => {
    if (!mapInstanceRef.current || !gps) return;

    const { latitude, longitude } = gps;
    const heading = odom?.heading_deg + 125 || 0;

    // update old vision cone
    if (visionConeRef.current) {
      mapInstanceRef.current.removeLayer(visionConeRef.current);
    }

    const conePoints = createVisionCone(
      latitude,
      longitude,
      heading,
      0.0003, // radius
      45 // cone angle
    );

    visionConeRef.current = L.polygon(conePoints, {
      color: "#4285f4",
      fillColor: "#4285f4",
      fillOpacity: 0.2,
      weight: 2,
      opacity: 0.4,
    }).addTo(mapInstanceRef.current);

    if (!markerRef.current) {
      // custom icon for robot
      const robotIcon = L.divIcon({
        className: "robot-marker",
        html: `
          <div class="robot-marker__container">
            <div class="robot-marker__dot"></div>
            <div class="robot-marker__arrow" style="transform: rotate(${heading}deg)">
              <svg width="24" height="24" viewBox="0 0 24 24">
                <path d="M12 2 L12 16 M12 2 L8 6 M12 2 L16 6" 
                      stroke="white" 
                      stroke-width="2.5" 
                      fill="none" 
                      stroke-linecap="round"/>
              </svg>
            </div>
          </div>
        `,
        iconSize: [40, 40],
        iconAnchor: [20, 20],
      });

      markerRef.current = L.marker([latitude, longitude], {
        icon: robotIcon,
        zIndexOffset: 1000,
      }).addTo(mapInstanceRef.current);
    } else {
      // update position
      markerRef.current.setLatLng([latitude, longitude]);

      // update heading
      const markerElement = markerRef.current.getElement();
      if (markerElement) {
        const arrowElement = markerElement.querySelector(
          ".robot-marker__arrow"
        );
        if (arrowElement) {
          arrowElement.style.transform = `rotate(${heading}deg)`;
        }
      }
    }

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
          <span className="map-center-btn-glass__icon">Center on robot</span>
        </button>
      )}

      {isFollowing && <div className="map-following-badge">Following</div>}
    </div>
  );
}
