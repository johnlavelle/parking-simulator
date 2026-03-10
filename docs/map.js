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

    addCar(corners, color, label) {
        const latlngs = corners.map(c => [c[1], c[0]]);
        const polygon = L.polygon(latlngs, {
            color: color || CONFIG.COLORS.carStroke,
            fillColor: color || CONFIG.COLORS.car,
            fillOpacity: 0.7,
            weight: 2,
        });
        this.carLayers.addLayer(polygon);
        return polygon;
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
