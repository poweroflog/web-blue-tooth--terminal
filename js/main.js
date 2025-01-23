class kalmanFilter {
  /*
  칼먼 필터

  RSSI값을 보정하기 위해 사용
  일반적으로 모두 내부의 값

  .flitering으로 값 업데이트
  .getRSSI로 현재 RSSI값 가져옴

  */
  constructor(processNoise = Number(1e-3), measurementNoise = 1) {
    this.initialized = false;
    this.processNoise = processNoise;
    this.measurementNoise = measurementNoise;
    this.predictedRSSI = 998244353;
    this.prevRSSI = 1;
    this.prevErrorCovariance = 0;
    this.predictedErrorCovariance = 0;
  }

  filtering(rssi) {
    // 칼먼 필터링. rssi = 측정된 RSSI 값
    if (!this.initialized) {
      this.initialized = true;
      this.prevRSSI = rssi;
      this.prevErrorCovariance = 1;
    } else {
      this.prevRSSI = this.predictedRSSI;
      this.predictedErrorCovariance =
        this.prevErrorCovariance + this.processNoise;

      const kalmanGain =
        this.predictedErrorCovariance /
        (this.predictedErrorCovariance + this.measurementNoise);
      this.predictedRSSI =
        this.prevRSSI > 0
          ? rssi
          : this.prevRSSI + kalmanGain * (rssi - this.prevRSSI);
      this.prevErrorCovariance =
        (1 - kalmanGain) * this.predictedErrorCovariance;
    }

    // this.predictedRSSI = rssi;
  }

  getRSSI() {
    // RSSI값 반환
    return this.predictedRSSI;
  }
}

// 상수들
// 기본적으로 4개 사이즈를 쓴다 가정하고 앵커 개수와 포지션은 미리 임의로 설정
const fetchUrl = "https://127.0.0.1";
const anchorSize = 4;
const anchorPos = [
  { x: 0, y: 0, txPower: -42  },
  { x: 0, y: 1000, txPower: -42 },
  { x: 1000, y: 0, txPower: -42 },
  { x: 1000, y: 1000, txPower: -42 },
];
const kalmanFilters = [];
for (let i = 0; i < anchorSize; i++) {
  kalmanFilters.push(new kalmanFilter());
}

// BLE 스캔
let scan = null;
let scanOn = false;
let scanInterval = 0;
let sendInterval = 0;

// 실제로 Front-end 버튼과 연결할 함수
// 임시로 모바일 터미널 디버깅을 위해 logToTerminal을 console.log 대신 쓰고 있음
async function toggleSyncBLEAnchors() {
  // 버튼 클릭 시 스캔 토글
  scanOn = !scanOn;
  if (!scanOn) {
    logToTerminal("Stopping scan...");
    if (scan != null) scan.stop();
    logToTerminal("Stopped.  scan.active = " + scan.active);
    return;
  }

  let filters = [];

  filters.push({ namePrefix: "M09" });

  let options = {};
  options.filters = filters;

  try {
    logToTerminal(
      "Requesting Bluetooth Scan with options: " + JSON.stringify(options)
    );
    scan = await navigator.bluetooth.requestLEScan(options);

    logToTerminal("Scan started with:");
    logToTerminal(" acceptAllAdvertisements: " + scan.acceptAllAdvertisements);
    logToTerminal(" active: " + scan.active);
    logToTerminal(" keepRepeatedDevices: " + scan.keepRepeatedDevices);
    logToTerminal(" filters: " + JSON.stringify(scan.filters));

    navigator.bluetooth.addEventListener("advertisementreceived", (event) => {
      const idx = Number(event.device.name.replace("M09-", ""));
      kalmanFilters[idx].filtering(Number(event.rssi));
    });

    // 포지션 로깅
    scanInterval = setInterval(() => {
      if (!scanOn) {
        clearInterval(scanInterval);
        return;
      }
      logToTerminal(JSON.stringify(getPosition()));
    }, 1000);

    sendInterval = setInterval(sendPosition(), 100);
  } catch (error) {
    console.log("Error: " + error);
  }
}

const inverseMatrix2x2 = (mat) => {
  // 역행렬 계산. mat = 형렬
  if (mat.length !== 2 || mat[0].length !== 2) {
    throw new Error("Matrix must be 2x2");
  }

  const ret = [
    [0, 0],
    [0, 0],
  ];
  const determiant = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];

  if (determiant == 0) {
    throw new Error("Matrix is not invertible");
  }

  ret[0][0] = (1.0 * mat[1][1]) / determiant;
  ret[1][0] = (1.0 * -mat[0][1]) / determiant;
  ret[0][1] = (1.0 * -mat[1][0]) / determiant;
  ret[1][1] = (1.0 * mat[0][0]) / determiant;

  return ret;
};

const transposeMatrix = (mat) => {
  // 행렬 Transpose 계산. mat = 행렬
  if (mat.length === 0) {
    throw new Error("Transposing matrix must not be empty.");
  }

  return mat[0].map((_, colIndex) => mat.map((row) => row[colIndex]));
};

const multiplyMatrix = (m1, m2) => {
  // 행렬 곱 계산. m1, m2는 각각 행렬이며, m1의 열 수가  m2의 행 수와 일치해야 함.
  if (m1.length < 1 || m1[0].length !== m2.length) {
    throw new Error(
      `Cannot multiply matrix! ${JSON.stringify(m1)}, ${JSON.stringify(m2)}`
    );
  }

  const ret = new Array(m1.length)
    .fill(0)
    .map(() => new Array(m2[0].length).fill(0));

  for (let i = 0; i < m1.length; i++) {
    for (let j = 0; j < m2[0].length; j++) {
      for (let k = 0; k < m2.length; k++) {
        ret[i][j] += m1[i][k] * m2[k][j];
      }
    }
  }

  return ret;
};

const calculateDistance = (rssi, txPower, pathLossExponent = 2) => {
  // 거리 계산 함수. RSSI와 txPower에서 cm 단위 거리 반환
  return Math.pow(10, (txPower - rssi) / (10 * pathLossExponent)) * 100;
};

function getPosition() {
  // 위치 측정해서 좌표를 반환
  // 삼변 측량법
  const m1 = [];
  const m2 = [];

  const dist = new Array(anchorSize).fill(0);

  // RSSI로부터 거리 계산
  for (let i = 0; i < anchorSize; i++) {
    dist[i] = calculateDistance(
      kalmanFilters[i].getRSSI(),
      anchorPos[i].txPower,
      4
    );
  }

  logToTerminal(
    `rssi: [ ${kalmanFilters[0].getRSSI()}, ${kalmanFilters[1].getRSSI()}, ${kalmanFilters[2].getRSSI()}, ${kalmanFilters[3].getRSSI()} ]`
  );
  logToTerminal(`dist: [ ${dist[0]}, ${dist[1]}, ${dist[2]}, ${dist[3]} ]`);

  // 기본 행렬 m1, m2 제작 후 행렬 계산
  for (let i = 1; i < anchorSize; i++) {
    m1.push([anchorPos[i].x - anchorPos[0].x, anchorPos[i].y - anchorPos[0].y]);
    m2.push([
      (Math.pow(anchorPos[i].x, 2) +
        Math.pow(anchorPos[i].y, 2) -
        Math.pow(dist[i], 2) -
        (Math.pow(anchorPos[0].x, 2) +
          Math.pow(anchorPos[0].y, 2) -
          Math.pow(dist[0], 2))) /
        2,
    ]);
  }

  const transM1 = transposeMatrix(m1);
  const position = multiplyMatrix(
    multiplyMatrix(inverseMatrix2x2(multiplyMatrix(transM1, m1)), transM1),
    m2
  );

  return [position[0][0], position[1][0]];
}

function sendPosition() {
  // 서버와 통신해서 좌표를 Post하는 함수
  if (!scanOn) {
    clearInterval(sendInterval);
    return;
  }
  fetch(fetchUrl, {
    method: "POST",
    body: JSON.stringify({
      pos: getPosition(),
    }),
    headers: {
      "Content-type": "application/json",
    },
  }).then((response) => {});
}

// 이 아래부터는 실제 프로덕션엔 필요없는 내용

// UI
const deviceNameLabel = document.getElementById("device-name");
const toolbarContainer = document.getElementById("toolbar");
const terminalContainer = document.getElementById("terminal");
const sendForm = document.getElementById("send-form");
const inputField = document.getElementById("input");

// Helpers
const defaultDeviceName = "Terminal";
const terminalAutoScrollingLimit = terminalContainer.offsetHeight / 2;
let isTerminalAutoScrolling = true;

const scrollElement = (element) => {
  const scrollTop = element.scrollHeight - element.offsetHeight;

  if (scrollTop > 0) {
    element.scrollTop = scrollTop;
  }
};

const logToTerminal = (message, type = "") => {
  terminalContainer.insertAdjacentHTML(
    "beforeend",
    `<div${type && ` class="${type}"`}>${message}</div>`
  );

  if (isTerminalAutoScrolling) {
    scrollElement(terminalContainer);
  }
};

const connectBtn = document.getElementById("connect");
connectBtn.addEventListener("click", toggleSyncBLEAnchors);

sendForm.addEventListener("submit", (event) => {
  event.preventDefault();

  // send(inputField.value);

  inputField.value = "";
  inputField.focus();
});

// Switch terminal auto scrolling if it scrolls out of bottom.
terminalContainer.addEventListener("scroll", () => {
  const scrollTopOffset =
    terminalContainer.scrollHeight -
    terminalContainer.offsetHeight -
    terminalAutoScrollingLimit;

  isTerminalAutoScrolling = scrollTopOffset < terminalContainer.scrollTop;
});
