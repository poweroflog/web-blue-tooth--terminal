class KalmanFilter {
  constructor(processNoise = 0.005, measurementNoise = 20) {
		this.initialized = false;
		this.processNoise = processNoise;
		this.measurementNoise = measurementNoise;
		this.predictedRSSI = 1;
    this.priorRSSI = 1;
		this.errorCovariance = 0;
    this.priorErrorCovariance = 0;
  }

	filtering(rssi) {
		if (!this.initialized) {
			this.initialized = true;
			this.priorRSSI = rssi;
			this.priorErrorCovariance = 1;
    }
		else {
			this.priorRSSI = this.predictedRSSI;
			this.priorErrorCovariance = this.errorCovariance + this.processNoise;

		  const kalmanGain = this.priorErrorCovariance / (this.priorErrorCovariance + this.measurementNoise);
		  this.predictedRSSI = this.priorRSSI == 1 ? rssi : this.priorRSSI + (kalmanGain * (rssi - this.priorRSSI));
		  this.errorCovarianceRSSI = (1 - kalmanGain) * this.priorErrorCovariance;
    }
  }

  getRSSI() {
    return this.predictedRSSI;
  }
}

const beaconSize = 4;
const timer = ms => new Promise(res => setTimeout(res, ms));

// UI elements.
const deviceNameLabel = document.getElementById('device-name');
// const connectButton = document.getElementById('connect');
// const disconnectButton = document.getElementById('disconnect');
const toolbarContainer = document.getElementById('toolbar');
const terminalContainer = document.getElementById('terminal');
const sendForm = document.getElementById('send-form');
const inputField = document.getElementById('input');

// Helpers.
const defaultDeviceName = 'Terminal';
const terminalAutoScrollingLimit = terminalContainer.offsetHeight / 2;
let isTerminalAutoScrolling = true;

const scrollElement = (element) => {
  const scrollTop = element.scrollHeight - element.offsetHeight;

  if (scrollTop > 0) {
    element.scrollTop = scrollTop;
  }
};

const logToTerminal = (message, type = '') => {
  terminalContainer.insertAdjacentHTML('beforeend',
    `<div${type && ` class="${type}"`}>${message}</div>`);

  if (isTerminalAutoScrolling) {
    scrollElement(terminalContainer);
  }
};

const anchor_pos = [{x: 0, y: 0}, {x: 0, y: 1000}, {x: 1000, y: 0}, {x: 1000, y: 1000}];
const rssi = [new KalmanFilter(), new KalmanFilter(), new KalmanFilter(), new KalmanFilter(),];
let scan = null;
let scanOn = false;

async function onButtonClick() {
  scanOn = !scanOn;
  if (!scanOn) {
    logToTerminal('Stopping scan...');
    scan.stop();
    logToTerminal('Stopped.  scan.active = ' + scan.active);
    return;
  }

  let filters = [];

  filters.push({namePrefix: 'M09'});

  let options = {};
  options.filters = filters;

  try {
    logToTerminal('Requesting Bluetooth Scan with options: ' + JSON.stringify(options));
    scan = await navigator.bluetooth.requestLEScan(options);

    logToTerminal('Scan started with:');
    logToTerminal(' acceptAllAdvertisements: ' + scan.acceptAllAdvertisements);
    logToTerminal(' active: ' + scan.active);
    logToTerminal(' keepRepeatedDevices: ' + scan.keepRepeatedDevices);
    logToTerminal(' filters: ' + JSON.stringify(scan.filters));

    navigator.bluetooth.addEventListener('advertisementreceived', event => {
      const idx = Number(event.device.name.replace('M09-', ''));
      rssi[idx].filtering(Number(event.rssi));
      logToTerminal(`got rssi: [ ${rssi[0].getRSSI()}, ${rssi[1].getRSSI()}, ${rssi[2].getRSSI()}, ${rssi[3].getRSSI()} ]`);
      logToTerminal(getPosition());
    });
  } catch(error)  {
    log('ERROR: ' + error);
  }
}

const inverseMatrix2d = mat => {  // 역행렬
  const ret = [[0, 0], [0, 0]];
  const determiant = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];

  ret[0][0] = 1.0 * mat[1][1] / determiant;
  ret[0][1] = 1.0 * -mat[0][1] / determiant;
  ret[0][1] = 1.0 * -mat[1][0] / determiant;
  ret[1][1] = 1.0 * mat[0][0] / determiant;

  return ret;
};

const transposeMatrix = mat => {  // 행렬 변환
  try {
    const ret = [];
    for (let j = 0; j < mat[0].length; j++) {
      const tmp = [];
      for (let i = 0; i < mat.length; i++) {
        tmp.push(mat[i][j]);
      }
      ret.push(tmp);
    }

    return ret;
  } catch (error) {
    console.log("fdf");
    return [];
  }
};

const multiplyMatrix = (m1, m2) => { // 행렬 곱
    try {

      const ret = [];
      console.log(m1, m2, m1.length, m2.length);
      for (let i = 0; i < m1.length; i++) {
        const tmp = [];
        for (let j = 0; j < m2[0].length; j++) {
          let cur = 0;
          for (let k = 0; k < m2.length; k++) {
            cur += m1[i][k] * m2[k][j];
          }
          tmp.push(cur);
        }
        ret.push(tmp);
      }

      console.log(ret);
      return ret;
    } catch (error) {
      logToTerminal(`Error multiplying matrix: ${error}`);
      return [];
    }
}

function getPosition() { // 위치 측정
  // 삼변 측량법
  const m1 = [];
  const m2 = [];

  const dist = [];
  for (let i = 2; i < beaconSize; i++) {
    dist.push(Math.max(0, (rssi[i].getRSSI() + 30) * -42));
  }

  for (let i = 2; i < 4; i++) {
    m1.push([anchor_pos[i].x - anchor_pos[1].x, anchor_pos[i].y - anchor_pos[1].y]);
    m2.push([
      Math.pow(anchor_pos[i].x, 2) + Math.pow(anchor_pos[i].y, 2) - Math.pow(dist[i], 2) - Math.pow(anchor_pos[1].x, 2) + Math.pow(anchor_pos[1].y, 2) - Math.pow(dist[1], 2)
    ])
  }

  const transM1 = transposeMatrix(m1);
  const position = multiplyMatrix(multiplyMatrix(inverseMatrix2d(multiplyMatrix(transM1, m1)), transM1), m2);

  return position;
}

const connectBtn = document.getElementById('connect');
connectBtn.addEventListener('click', onButtonClick);

sendForm.addEventListener('submit', (event) => {
  event.preventDefault();

  // send(inputField.value);

  inputField.value = '';
  inputField.focus();
});

// Switch terminal auto scrolling if it scrolls out of bottom.
terminalContainer.addEventListener('scroll', () => {
  const scrollTopOffset = terminalContainer.scrollHeight -
    terminalContainer.offsetHeight - terminalAutoScrollingLimit;

  isTerminalAutoScrolling = (scrollTopOffset < terminalContainer.scrollTop);
});