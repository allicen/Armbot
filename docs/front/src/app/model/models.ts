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
