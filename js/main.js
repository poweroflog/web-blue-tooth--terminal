class kalmanFilter {
  /*
  칼먼 필터

  RSSI값을 보정하기 위해 사용
  일반적으로 모두 내부의 값

  .flitering으로 값 업데이트
  .getRSSI로 현재 RSSI값 가져옴

  */
  constructor(processNoise = 0.005, measurementNoise = 20) {
    this.initialized = false;
    this.processNoise = processNoise;
    this.measurementNoise = measurementNoise;
    this.predictedRSSI = 1;
    this.priorRSSI = 1;
    this.errorCovariance = 0;
    this.priorErrorCovariance = 0;
  }

  filtering(rssi) { // 칼먼 필터링. rssi = 측정된 RSSI 값
    // if (!this.initialized) {
    // 	this.initialized = true;
    // 	this.priorRSSI = rssi;
    // 	this.priorErrorCovariance = 1;
    // }
    // else {
    // 	this.priorRSSI = this.predictedRSSI;
    // 	this.priorErrorCovariance = this.errorCovariance + this.processNoise;

    //   const kalmanGain = this.priorErrorCovariance / (this.priorErrorCovariance + this.measurementNoise);
    //   this.predictedRSSI = this.priorRSSI == 1 ? rssi : this.priorRSSI + (kalmanGain * (rssi - this.priorRSSI));
    //   this.errorCovarianceRSSI = (1 - kalmanGain) * this.priorErrorCovariance;
    // }
    this.predictedRSSI = rssi;
  }

  getRSSI() { // RSSI값 반환
    return this.predictedRSSI;
  }
}

// 상수들
// 기본적으로 4개 사이즈를 쓴다 가정하고 앵커 개수와 포지션은 미리 임의로 설정
const fetchUrl = "";
const anchorSize = 4;
const anchorPos = [{ x: 0, y: 0 }, { x: 0, y: 1000 }, { x: 1000, y: 0 }, { x: 1000, y: 1000 }];
const kalmanFilters = [];
for (let i = 0; i < anchorSize; i++) {
  kalmanFilters.push(new kalmanFilter());
}

// BLE 스캔
let scan = null;
let scanOn = false;

// 실제로 Front-end 버튼과 연결할 함수
// 임시로 모바일 터미널 디버깅을 위해 logToTerminal을 console.log 대신 쓰고 있음
async function toggleSyncBLEAnchors() { // 버튼 클릭 시 스캔 토글
  scanOn = !scanOn;
  if (!scanOn) {
    logToTerminal('Stopping scan...');
    scan.stop();
    logToTerminal('Stopped.  scan.active = ' + scan.active);
    return;
  }

  let filters = [];

  filters.push({ namePrefix: 'M09' });

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
      kalmanFilters[idx].filtering(Number(event.rssi));
      logToTerminal(`got rssi: [ ${kalmanFilters[0].getRSSI()}, ${kalmanFilters[1].getRSSI()}, ${kalmanFilters[2].getRSSI()}, ${kalmanFilters[3].getRSSI()} ]`);
      logToTerminal(`pos: ${JSON.stringify(getPosition())}`);
    });

    sendPosition();
  } catch (error) {
    console.log('Error getting BLE Anchors: ' + error);
  }
}

const inverseMatrix2d = mat => {  // 역행렬 계산. mat = 형렬
  const ret = [[0, 0], [0, 0]];
  const determiant = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];

  ret[0][0] = 1.0 * mat[1][1] / determiant;
  ret[0][1] = 1.0 * -mat[0][1] / determiant;
  ret[0][1] = 1.0 * -mat[1][0] / determiant;
  ret[1][1] = 1.0 * mat[0][0] / determiant;

  return ret;
};

const transposeMatrix = mat => {  // 행렬 Transpose 계산. mat = 행렬
  const ret = [];
  try {
    for (let j = 0; j < mat[0].length; j++) {
      const tmp = [];
      for (let i = 0; i < mat.length; i++) {
        tmp.push(mat[i][j]);
      }
      ret.push(tmp);
    }
  } catch (error) {
    console.log(`Error transposing matrix: ${error}`);
  }

  return ret;
};

const multiplyMatrix = (m1, m2) => { // 행렬 곱 계산. m1, m2는 각각 행렬이며, m1의 열 수가  m2의 행 수와 일치해야 함.
  const ret = [];
  try {
    const ret = [];
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
  } catch (error) {
    console.log(`Error multiplying matrix: ${error}`);
  }

  return ret;
}

function getPosition() { // 위치 측정해서 좌표를 반환
  // 삼변 측량법
  const m1 = [];
  const m2 = [];

  const dist = [];
  for (let i = 0; i < anchorSize; i++) {
    dist.push(0);
  }
  // RSSI로부터 거리 계산
  for (let i = 2; i < anchorSize; i++) {
    const curRSSI = kalmanFilters[i].getRSSI();
    dist[i] = Math.max(0, -(curRSSI + 30) * 42);
  }
  // 기본 행렬 m1, m2 제작 후 행렬 계산
  for (let i = 2; i < anchorSize; i++) {
    m1.push([anchorPos[i].x - anchorPos[1].x, anchorPos[i].y - anchorPos[1].y]);
    m2.push(
      [
        Math.pow(anchorPos[i].x, 2) + Math.pow(anchorPos[i].y, 2) - Math.pow(dist[i], 2) - Math.pow(anchorPos[1].x, 2) + Math.pow(anchorPos[1].y, 2) - Math.pow(dist[1], 2)
      ]
    )
  }

  const transM1 = transposeMatrix(m1);
  const position = multiplyMatrix(multiplyMatrix(inverseMatrix2d(multiplyMatrix(transM1, m1)), transM1), m2);

  return [position[0][0], position[1][0]];
}

function sendPosition() { // 서버와 통신해서 좌표를 Post하는 함수
  if (!scanOn)
    return;
  fetch(fetchUrl, {
    method: "POST",
    body: JSON.stringify({
      pos: getPosition(),
    }),
    headers: {
      "Content-type": "application/json",
    },
  }).then(() => setTimeout(sendPosition(), 100));
}

// 이 아래부터는 실제 프로덕션엔 필요없는 내용

// UI
const deviceNameLabel = document.getElementById('device-name');
const toolbarContainer = document.getElementById('toolbar');
const terminalContainer = document.getElementById('terminal');
const sendForm = document.getElementById('send-form');
const inputField = document.getElementById('input');

// Helpers
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

const connectBtn = document.getElementById('connect');
connectBtn.addEventListener('click', toggleSyncBLEAnchors);

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