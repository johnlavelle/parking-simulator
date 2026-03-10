class Simulation {
    constructor(parkingMap) {
        this.parkingMap = parkingMap;
        this.carEntries = [];
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
        this.carEntries = [];
        this.placements = placements;
        this.carLength = carLength;
        this.carWidth = carWidth;

        placements.forEach((p, i) => {
            const color = CONFIG.CAR_COLORS[i % CONFIG.CAR_COLORS.length];
            const [cx, cy] = wgs84ToLocal(p.center[0], p.center[1]);
            const heading = p.heading * Math.PI / 180;
            const parts = this._getCarParts(cx, cy, heading, carLength, carWidth);
            const layers = this.parkingMap.addCar(parts, color);
            this.carEntries.push({ layers, placement: p, color });
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

        this.parkingMap.clearPaths();
        this.paths.forEach((path) => {
            const carEntry = this.carEntries.find(c => c.placement.car_index === path.car_index);
            const color = carEntry ? carEntry.color : CONFIG.COLORS.path;
            this.parkingMap.addPathLine(path.waypoints, color);
        });

        for (const path of this.paths) {
            const carEntry = this.carEntries.find(c => c.placement.car_index === path.car_index);
            if (!carEntry) continue;
            await this._replayTrajectory(carEntry, path.trajectory);
            this.parkingMap.removeCar(carEntry.layers);
        }

        this.parkingMap.clearPaths();
        this.animating = false;
        if (onComplete) onComplete();
    }

    _replayTrajectory(carEntry, trajectory) {
        return new Promise(resolve => {
            if (!trajectory || trajectory.length < 2) { resolve(); return; }

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

                const parts = this._getCarParts(cx, cy, heading, L, W);
                this.parkingMap.updateCar(carEntry.layers, parts);

                frameIdx++;
                setTimeout(tick, CONFIG.ANIMATION.frameMs);
            };

            tick();
        });
    }

    /**
     * Generate all car sub-shapes in WGS84 [lon, lat] coords.
     * Local car coords: origin at center, x+ = front, y+ = left.
     */
    _getCarParts(cx, cy, heading, length, width) {
        const cos_h = Math.cos(heading);
        const sin_h = Math.sin(heading);
        const hl = length / 2;
        const hw = width / 2;

        // Transform local car coord to world
        const tx = (lx, ly) => {
            const wx = cx + lx * cos_h - ly * sin_h;
            const wy = cy + lx * sin_h + ly * cos_h;
            return localToWgs84(wx, wy);
        };

        // Body outline — tapered front, flat rear, slight corner rounding
        const body = [
            // Rear (left to right)
            tx(-hl,         hw * 0.88),
            tx(-hl * 0.94,  hw),
            // Left side
            tx( hl * 0.30,  hw),
            // Front-left taper
            tx( hl * 0.60,  hw * 0.95),
            tx( hl * 0.80,  hw * 0.82),
            tx( hl * 0.92,  hw * 0.55),
            // Nose
            tx( hl,         0),
            // Front-right taper
            tx( hl * 0.92, -hw * 0.55),
            tx( hl * 0.80, -hw * 0.82),
            tx( hl * 0.60, -hw * 0.95),
            // Right side
            tx( hl * 0.30, -hw),
            tx(-hl * 0.94, -hw),
            // Rear right
            tx(-hl,        -hw * 0.88),
        ];

        // Cabin / roof (darker overlay in the middle section)
        const cabin = [
            tx(-hl * 0.40,  hw * 0.82),
            tx( hl * 0.30,  hw * 0.82),
            tx( hl * 0.30, -hw * 0.82),
            tx(-hl * 0.40, -hw * 0.82),
        ];

        // Windshield (front window)
        const windshield = [
            tx( hl * 0.30,  hw * 0.82),
            tx( hl * 0.48,  hw * 0.72),
            tx( hl * 0.48, -hw * 0.72),
            tx( hl * 0.30, -hw * 0.82),
        ];

        // Rear window
        const rearWindow = [
            tx(-hl * 0.40,  hw * 0.80),
            tx(-hl * 0.55,  hw * 0.70),
            tx(-hl * 0.55, -hw * 0.70),
            tx(-hl * 0.40, -hw * 0.80),
        ];

        // Headlights (two small quads at front)
        const hlSize = hl * 0.06;
        const hlW = hw * 0.14;
        const headlightL = [
            tx(hl * 0.88,  hw * 0.62 + hlW),
            tx(hl * 0.88 + hlSize, hw * 0.55 + hlW),
            tx(hl * 0.88 + hlSize, hw * 0.55),
            tx(hl * 0.88,  hw * 0.62),
        ];
        const headlightR = [
            tx(hl * 0.88, -hw * 0.62),
            tx(hl * 0.88 + hlSize, -hw * 0.55),
            tx(hl * 0.88 + hlSize, -hw * 0.55 - hlW),
            tx(hl * 0.88, -hw * 0.62 - hlW),
        ];

        // Tail lights (two small quads at rear)
        const tlSize = hl * 0.04;
        const tlW = hw * 0.14;
        const taillightL = [
            tx(-hl + tlSize,  hw * 0.88),
            tx(-hl,           hw * 0.88),
            tx(-hl,           hw * 0.88 - tlW),
            tx(-hl + tlSize,  hw * 0.88 - tlW),
        ];
        const taillightR = [
            tx(-hl + tlSize, -hw * 0.88 + tlW),
            tx(-hl,          -hw * 0.88 + tlW),
            tx(-hl,          -hw * 0.88),
            tx(-hl + tlSize, -hw * 0.88),
        ];

        return {
            body, cabin, windshield, rearWindow,
            headlightL, headlightR,
            taillightL, taillightR,
        };
    }
}
