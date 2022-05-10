export interface Coordinate {
  id: number;
  name: string;
  x: number;
  y: number;
  z: number;
}

export interface LaunchFileRow {
  id: number,
  coordinate: Coordinate,
  delay: number,
  sortOrder: number
}

export interface IdData {
  type: string;
  title: string;
  text: string;
  id: 0;
}

export interface Response {
  status: string
}

export interface Position {
  x: number,
  y: number
}

export interface WorkOption {
  key: string,
  value: string
}

export interface Motor {
  key: number,
  value: string,
  forwardDirection: string,
  inverseDirection: string
}

export interface CameraImage {
  data: string | null,
  encoding: string | null,
  width: number,
  height: number
}

export interface RobotInfoFromCamera {
  resultExists: boolean | null; // Результат пришел
  info: string | null,
  isStartPosition: boolean | null,
  xError: number | null,
  yError: number | null,
  zError: number | null,
  joint1: number | null,
  joint2: number | null,
  joint3: number | null,
  joint4: number | null
}
