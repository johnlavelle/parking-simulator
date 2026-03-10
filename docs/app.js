document.addEventListener('DOMContentLoaded', () => {
    const parkingMap = new ParkingMap('map');
    parkingMap.loadPolygons();

    const sim = new Simulation(parkingMap);

    const carCountSlider = document.getElementById('car-count');
    const carCountDisplay = document.getElementById('car-count-display');
    const carTypeSelect = document.getElementById('car-type');
    const simulateBtn = document.getElementById('simulate-btn');
    const driveOutBtn = document.getElementById('drive-out-btn');
    const warningsDiv = document.getElementById('warnings');
    const rowWidthInfo = document.getElementById('row-width-info');

    // Load config from embedded data
    const cfg = SIM_DATA.config;

    carCountSlider.max = cfg.max_cars;
    if (parseInt(carCountSlider.value) > cfg.max_cars) {
        carCountSlider.value = cfg.max_cars;
        carCountDisplay.textContent = cfg.max_cars;
    }

    carTypeSelect.innerHTML = '';
    cfg.car_types.forEach((ct) => {
        const opt = document.createElement('option');
        opt.value = ct.value;
        opt.textContent = ct.label;
        if (ct.value === 'Medium') opt.selected = true;
        carTypeSelect.appendChild(opt);
    });

    rowWidthInfo.textContent =
        `Narrowest ROW: ${cfg.row_min_width}m (max car width: ${cfg.max_car_width}m)`;

    carCountSlider.addEventListener('input', () => {
        carCountDisplay.textContent = carCountSlider.value;
    });

    function showWarnings(warnings) {
        warningsDiv.innerHTML = warnings
            .map(w => `<div class="warning">${w}</div>`)
            .join('');
    }

    simulateBtn.addEventListener('click', () => {
        simulateBtn.disabled = true;
        simulateBtn.textContent = 'Loading...';
        driveOutBtn.disabled = true;
        warningsDiv.innerHTML = '';
        parkingMap.clearCars();
        parkingMap.clearPaths();

        const numCars = parseInt(carCountSlider.value);
        const carType = carTypeSelect.value;
        const key = `${carType}_${numCars}`;

        const data = SIM_DATA.simulations[key];

        if (!data) {
            showWarnings([`No simulation data for ${numCars} ${carType} cars`]);
        } else {
            sim.setCars(data.placements, data.car_length, data.car_width);
            sim.setPaths(data.paths);
            showWarnings(data.warnings);

            if (data.placements.length > 0) {
                driveOutBtn.disabled = false;
            }
        }

        simulateBtn.disabled = false;
        simulateBtn.textContent = 'Simulate Parking';
    });

    driveOutBtn.addEventListener('click', () => {
        if (sim.animating) {
            const paused = sim.togglePause();
            driveOutBtn.textContent = paused ? 'Resume' : 'Pause';
        } else {
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
