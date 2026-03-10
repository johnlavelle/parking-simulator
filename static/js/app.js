document.addEventListener('DOMContentLoaded', async () => {
    const parkingMap = new ParkingMap('map');
    await parkingMap.loadPolygons();

    const sim = new Simulation(parkingMap);

    const carCountSlider = document.getElementById('car-count');
    const carCountDisplay = document.getElementById('car-count-display');
    const carTypeSelect = document.getElementById('car-type');
    const simulateBtn = document.getElementById('simulate-btn');
    const driveOutBtn = document.getElementById('drive-out-btn');
    const warningsDiv = document.getElementById('warnings');
    const rowWidthInfo = document.getElementById('row-width-info');

    // Load config: allowed car types and max cars
    try {
        const configResp = await fetch('/api/config');
        const cfg = await configResp.json();

        carCountSlider.max = cfg.max_cars;
        if (parseInt(carCountSlider.value) > cfg.max_cars) {
            carCountSlider.value = cfg.max_cars;
            carCountDisplay.textContent = cfg.max_cars;
        }

        carTypeSelect.innerHTML = '';
        cfg.car_types.forEach((ct, i) => {
            const opt = document.createElement('option');
            opt.value = ct.value;
            opt.textContent = ct.label;
            if (ct.value === 'Medium') opt.selected = true;
            carTypeSelect.appendChild(opt);
        });

        rowWidthInfo.textContent =
            `Narrowest ROW: ${cfg.row_min_width}m (max car width: ${cfg.max_car_width}m)`;
    } catch (err) {
        console.error('Failed to load config:', err);
    }

    carCountSlider.addEventListener('input', () => {
        carCountDisplay.textContent = carCountSlider.value;
    });

    function showWarnings(warnings) {
        warningsDiv.innerHTML = warnings
            .map(w => `<div class="warning">${w}</div>`)
            .join('');
    }

    simulateBtn.addEventListener('click', async () => {
        simulateBtn.disabled = true;
        simulateBtn.textContent = 'Computing...';
        driveOutBtn.disabled = true;
        warningsDiv.innerHTML = '';
        parkingMap.clearCars();
        parkingMap.clearPaths();

        try {
            const resp = await fetch('/api/simulate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    num_cars: parseInt(carCountSlider.value),
                    car_type: carTypeSelect.value,
                }),
            });
            const data = await resp.json();

            if (data.detail) {
                showWarnings([data.detail]);
            } else {
                sim.setCars(data.placements, data.car_length, data.car_width);
                sim.setPaths(data.paths);
                showWarnings(data.warnings);

                if (data.placements.length > 0) {
                    driveOutBtn.disabled = false;
                }
            }
        } catch (err) {
            showWarnings([`Error: ${err.message}`]);
        }

        simulateBtn.disabled = false;
        simulateBtn.textContent = 'Simulate Parking';
    });

    driveOutBtn.addEventListener('click', () => {
        if (sim.animating) {
            // Toggle pause/resume
            const paused = sim.togglePause();
            driveOutBtn.textContent = paused ? 'Resume' : 'Pause';
        } else {
            // Start animation
            driveOutBtn.textContent = 'Pause';
            simulateBtn.disabled = true;
            sim.animateDriveOut(() => {
                simulateBtn.disabled = false;
                driveOutBtn.disabled = true;
                driveOutBtn.textContent = 'Drive Out';
            });
        }
    });
});
