const CONFIG = {
    MAP_CENTER: [54.6426, -5.8354],
    MAP_ZOOM: 20,
    COLORS: {
        building: '#808080',
        parkingArea: '#4CAF50',
        rightOfWay: '#FFC107',
        car: '#2196F3',
        carStroke: '#1565C0',
        path: '#FF5722',
    },
    ANIMATION: {
        frameMs: 30,         // ms between render frames
        speed: 6.0,          // m/s vehicle speed (sped up for UX)
        substeps: 4,         // physics substeps per frame
        lookAheadFactor: 2.5, // look-ahead = wheelbase * this
        maxSteeringDeg: 38,  // max front-wheel steering angle
        wheelbaseFactor: 0.6, // wheelbase = car_length * this
        rearAxleRatio: 0.25, // rear axle at 25% from back of car
        arrivalDist: 1.5,    // meters from end to stop
    },
    // Equirectangular projection constants at lat ~54.64
    COORD: {
        REF_LON: -5.83543643,
        REF_LAT: 54.64258140,
        DEG_LON_TO_M: 64380,
        DEG_LAT_TO_M: 111320,
    },
    CAR_COLORS: [
        '#2196F3', '#E91E63', '#9C27B0', '#FF9800', '#00BCD4'
    ],
};

// Coordinate conversion helpers
function wgs84ToLocal(lon, lat) {
    return [
        (lon - CONFIG.COORD.REF_LON) * CONFIG.COORD.DEG_LON_TO_M,
        (lat - CONFIG.COORD.REF_LAT) * CONFIG.COORD.DEG_LAT_TO_M,
    ];
}

function localToWgs84(x, y) {
    return [
        x / CONFIG.COORD.DEG_LON_TO_M + CONFIG.COORD.REF_LON,
        y / CONFIG.COORD.DEG_LAT_TO_M + CONFIG.COORD.REF_LAT,
    ];
}
