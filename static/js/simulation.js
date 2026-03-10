class Simulation {
    constructor(parkingMap) {
        this.parkingMap = parkingMap;
        this.carPolygons = [];
        this.placements = [];
        this.paths = [];
        this.carLength = 4.3;
        this.carWidth = 1.8;
        this.animating = false;
        this.paused = false;
        this._pauseResolve = null;
    }

    setCars(placements, carLength, carWidth) {
        this.parkingMap.clearCars();
        this.parkingMap.clearPaths();
        this.carPolygons = [];
        this.placements = placements;
        this.carLength = carLength;
        this.carWidth = carWidth;

        placements.forEach((p, i) => {
            const color = CONFIG.CAR_COLORS[i % CONFIG.CAR_COLORS.length];
            const poly = this.parkingMap.addCar(p.corners, color);
            this.carPolygons.push({ polygon: poly, placement: p, color: color });
        });
    }

    setPaths(paths) {
        this.paths = paths;
    }

    togglePause() {
        if (!this.animating) return;
        this.paused = !this.paused;
        if (!this.paused && this._pauseResolve) {
            this._pauseResolve();
            this._pauseResolve = null;
        }
        return this.paused;
    }

    _waitIfPaused() {
        if (!this.paused) return Promise.resolve();
        return new Promise(resolve => { this._pauseResolve = resolve; });
    }

    async animateDriveOut(onComplete) {
        if (this.animating || this.paths.length === 0) return;
        this.animating = true;
        this.paused = false;

        // Show guide path lines
        this.parkingMap.clearPaths();
        this.paths.forEach((path) => {
            const carEntry = this.carPolygons.find(c => c.placement.car_index === path.car_index);
            const color = carEntry ? carEntry.color : CONFIG.COLORS.path;
            this.parkingMap.addPathLine(path.waypoints, color);
        });

        // Animate each car sequentially using pre-computed trajectory
        for (const path of this.paths) {
            const carEntry = this.carPolygons.find(c => c.placement.car_index === path.car_index);
            if (!carEntry) continue;
            await this._replayTrajectory(carEntry, path.trajectory);
            this.parkingMap.carLayers.removeLayer(carEntry.polygon);
        }

        this.parkingMap.clearPaths();
        this.animating = false;
        if (onComplete) onComplete();
    }

    /**
     * Replay a pre-computed trajectory. Each frame is [lon, lat, heading_deg].
     * The backend has already ensured the car stays within bounds.
     */
    _replayTrajectory(carEntry, trajectory) {
        return new Promise(resolve => {
            if (!trajectory || trajectory.length < 2) { resolve(); return; }

            const polygon = carEntry.polygon;
            const L = this.carLength;
            const W = this.carWidth;
            let frameIdx = 0;

            const tick = async () => {
                if (this.paused) {
                    await this._waitIfPaused();
                }

                if (frameIdx >= trajectory.length) {
                    resolve();
                    return;
                }

                const [lon, lat, headingDeg] = trajectory[frameIdx];
                const [cx, cy] = wgs84ToLocal(lon, lat);
                const heading = headingDeg * Math.PI / 180;

                const corners = this._getCarCorners(cx, cy, heading, L, W);
                const latlngs = corners.map(c => {
                    const [clon, clat] = localToWgs84(c[0], c[1]);
                    return [clat, clon];
                });
                polygon.setLatLngs(latlngs);

                frameIdx++;
                setTimeout(tick, CONFIG.ANIMATION.frameMs);
            };

            tick();
        });
    }

    /**
     * Compute the 4 corners of the car rectangle given center position and heading.
     */
    _getCarCorners(cx, cy, heading, length, width) {
        const cos = Math.cos(heading);
        const sin = Math.sin(heading);
        const hl = length / 2;
        const hw = width / 2;

        const localCorners = [
            { x: -hl, y:  hw },  // rear-left
            { x:  hl, y:  hw },  // front-left
            { x:  hl, y: -hw },  // front-right
            { x: -hl, y: -hw },  // rear-right
        ];

        return localCorners.map(c => [
            cx + c.x * cos - c.y * sin,
            cy + c.x * sin + c.y * cos,
        ]);
    }
}
