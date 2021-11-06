export interface Coordinate {
  id: number;
  name: string;
  x: number;
  y: number;
  z: number;
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
