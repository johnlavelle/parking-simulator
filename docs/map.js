class ParkingMap {
    constructor(containerId) {
        this.map = L.map(containerId).setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
        this.polygonLayers = L.layerGroup().addTo(this.map);
        this.carLayers = L.layerGroup().addTo(this.map);
        this.pathLayers = L.layerGroup().addTo(this.map);
        this._setupTileLayers();
    }

    _setupTileLayers() {
        const osm = L.tileLayer(
            'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
            { maxZoom: 22, maxNativeZoom: 19, attribution: '&copy; OpenStreetMap' }
        );
        const satellite = L.tileLayer(
            'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            { maxZoom: 22, maxNativeZoom: 19, attribution: '&copy; Esri' }
        );
        osm.addTo(this.map);
        L.control.layers(
            { 'Street': osm, 'Satellite': satellite },
            null,
            { position: 'topright' }
        ).addTo(this.map);
    }

    loadPolygons() {
        const data = SIM_DATA.polygons;
        this.polygonLayers.clearLayers();

        this._addGeoJSONPolygon(data.parking_area, {
            color: CONFIG.COLORS.parkingArea,
            fillColor: CONFIG.COLORS.parkingArea,
            fillOpacity: 0.3,
            weight: 2,
        });

        this._addGeoJSONPolygon(data.right_of_way, {
            color: CONFIG.COLORS.rightOfWay,
            fillColor: CONFIG.COLORS.rightOfWay,
            fillOpacity: 0.25,
            weight: 2,
        });

        this._addGeoJSONPolygon(data.building, {
            color: CONFIG.COLORS.building,
            fillColor: CONFIG.COLORS.building,
            fillOpacity: 0.6,
            weight: 2,
        });
    }

    _addGeoJSONPolygon(geojson, style) {
        const rings = geojson.coordinates.map(ring =>
            ring.map(c => [c[1], c[0]])
        );
        const polygon = L.polygon(rings, style);
        this.polygonLayers.addLayer(polygon);
    }

    /** Convert [lon, lat] array to Leaflet [lat, lon] */
    _toLatLng(coords) {
        return coords.map(c => [c[1], c[0]]);
    }

    _darken(hex, amount) {
        const num = parseInt(hex.replace('#', ''), 16);
        const r = Math.max(0, (num >> 16) - amount);
        const g = Math.max(0, ((num >> 8) & 0xFF) - amount);
        const b = Math.max(0, (num & 0xFF) - amount);
        return `rgb(${r},${g},${b})`;
    }

    /**
     * Add a multi-layer car shape. `parts` has: body, cabin, windshield,
     * rearWindow, headlightL, headlightR, taillightL, taillightR.
     * Returns an object with named layer references for updateCar().
     */
    addCar(parts, color) {
        const group = L.layerGroup();
        const stroke = this._darken(color, 60);

        const body = L.polygon(this._toLatLng(parts.body), {
            color: stroke,
            fillColor: color,
            fillOpacity: 0.85,
            weight: 1.5,
        });
        group.addLayer(body);

        const cabin = L.polygon(this._toLatLng(parts.cabin), {
            color: 'transparent',
            fillColor: this._darken(color, 35),
            fillOpacity: 0.6,
            weight: 0,
        });
        group.addLayer(cabin);

        const windshield = L.polygon(this._toLatLng(parts.windshield), {
            color: 'rgba(180,220,255,0.4)',
            fillColor: 'rgba(180,220,255,0.55)',
            fillOpacity: 0.55,
            weight: 0.5,
        });
        group.addLayer(windshield);

        const rearWindow = L.polygon(this._toLatLng(parts.rearWindow), {
            color: 'rgba(160,200,240,0.3)',
            fillColor: 'rgba(160,200,240,0.45)',
            fillOpacity: 0.45,
            weight: 0.5,
        });
        group.addLayer(rearWindow);

        const headlightL = L.polygon(this._toLatLng(parts.headlightL), {
            color: 'transparent',
            fillColor: '#FFFDE7',
            fillOpacity: 0.9,
            weight: 0,
        });
        group.addLayer(headlightL);

        const headlightR = L.polygon(this._toLatLng(parts.headlightR), {
            color: 'transparent',
            fillColor: '#FFFDE7',
            fillOpacity: 0.9,
            weight: 0,
        });
        group.addLayer(headlightR);

        const taillightL = L.polygon(this._toLatLng(parts.taillightL), {
            color: 'transparent',
            fillColor: '#D32F2F',
            fillOpacity: 0.9,
            weight: 0,
        });
        group.addLayer(taillightL);

        const taillightR = L.polygon(this._toLatLng(parts.taillightR), {
            color: 'transparent',
            fillColor: '#D32F2F',
            fillOpacity: 0.9,
            weight: 0,
        });
        group.addLayer(taillightR);

        this.carLayers.addLayer(group);

        return {
            group, body, cabin, windshield, rearWindow,
            headlightL, headlightR, taillightL, taillightR,
        };
    }

    /** Update all sub-layers of a car with new part coords. */
    updateCar(layers, parts) {
        layers.body.setLatLngs(this._toLatLng(parts.body));
        layers.cabin.setLatLngs(this._toLatLng(parts.cabin));
        layers.windshield.setLatLngs(this._toLatLng(parts.windshield));
        layers.rearWindow.setLatLngs(this._toLatLng(parts.rearWindow));
        layers.headlightL.setLatLngs(this._toLatLng(parts.headlightL));
        layers.headlightR.setLatLngs(this._toLatLng(parts.headlightR));
        layers.taillightL.setLatLngs(this._toLatLng(parts.taillightL));
        layers.taillightR.setLatLngs(this._toLatLng(parts.taillightR));
    }

    /** Remove a car's layer group. */
    removeCar(layers) {
        this.carLayers.removeLayer(layers.group);
    }

    clearCars() {
        this.carLayers.clearLayers();
    }

    clearPaths() {
        this.pathLayers.clearLayers();
    }

    addPathLine(waypoints, color) {
        const latlngs = waypoints.map(w => [w[1], w[0]]);
        const line = L.polyline(latlngs, {
            color: color || CONFIG.COLORS.path,
            weight: 2,
            dashArray: '5,5',
            opacity: 0.6,
        });
        this.pathLayers.addLayer(line);
        return line;
    }
}
