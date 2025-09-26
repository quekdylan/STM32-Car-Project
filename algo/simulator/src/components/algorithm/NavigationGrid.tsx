import { Position, Obstacle, Direction } from "../../schemas/entity";
import {
  GRID_TOTAL_HEIGHT,
  GRID_TOTAL_WIDTH,
  ROBOT_ACTUAL_GRID_HEIGHT,
  ROBOT_ACTUAL_GRID_WIDTH,
  ROBOT_GRID_HEIGHT,
  ROBOT_GRID_WIDTH,
} from "../../constants";
import { AlgoTestDataInterface } from "../../tests";
import { useEffect, useState } from "react";

interface NavigationGridProps {
  robotPosition: Position;
  robotPath: Position[] | undefined;
  obstacles: Obstacle[];
  canAddObstacle: boolean;
  setSelectedTest: React.Dispatch<React.SetStateAction<AlgoTestDataInterface>>;
}

const THEME = {
  bg: "#ffffff",              
  grid: "#1e1b4b",            
  axisText: "#000000",        
  labelText: "#f0abfc",       
  path: "#38618C",            
  robotFootprint: "#7dd3fc",  
  robotBody: "#afc1d6",       
  robotNorthEdgeIdle: "#655a7c",  
  robotNorthEdgeScan: "#691ba8",  
  obstacle: "#facc15",       
  obstacleScanned: "#88d3cf", 
  obstacleFacingEdge: "#515751",  
  snapMark: "#9197a1", 
  reversePath: "#f43f5e",
  reverseGlow: "#fb7185",        
};


export const NavigationGrid = (props: NavigationGridProps) => {
  const { robotPosition, robotPath, obstacles, canAddObstacle, setSelectedTest } = props;

  const cellSize = 40;
  const offset = 40; // space for axis labels

  const robotFootprintX = robotPosition.x * cellSize;
  const robotFootprintY = GRID_TOTAL_HEIGHT * cellSize - (robotPosition.y + 1) * cellSize - offset;
  const robotWidth = ROBOT_ACTUAL_GRID_WIDTH * cellSize;
  const robotHeight = ROBOT_ACTUAL_GRID_HEIGHT * cellSize;
  const robotCenterX = robotFootprintX + (cellSize * ROBOT_GRID_WIDTH - robotWidth) / 2;
  const robotCenterY = robotFootprintY + (cellSize * ROBOT_GRID_HEIGHT - robotHeight) / 2;

  // scanned obstacles state
  const [scannedObstacles, setScannedObstacles] = useState<Obstacle[]>([]);

  useEffect(() => {
    const scannedObstaclesAtStep: Obstacle[] = [];
    robotPath?.forEach((position) => {
      if (position?.s !== null) {
        const obstacleId = Number(position.s.split("_")[0]);
        const obstacle = obstacles.find((o) => o.id === obstacleId);
        if (obstacle) scannedObstaclesAtStep.push(obstacle);
      }
    });
    setScannedObstacles(scannedObstaclesAtStep);
  }, [robotPath, obstacles]);

  const getDirectionRotationAngle = (direction: Direction) => {
    switch (direction) {
      case Direction.NORTH:
        return 0;
      case Direction.EAST:
        return 90;
      case Direction.SOUTH:
        return 180;
      case Direction.WEST:
        return 270;
      default:
        return null;
    }
  };

  const robotAngle = getDirectionRotationAngle(robotPosition.d);

  const handleAddObstacle = (x: number, y: number, d: number) => {
    setSelectedTest((prev) => {
      const updated = {
        obstacles: [
          ...prev.obstacles,
          {
            id: prev.obstacles.length + 1,
            x,
            y,
            d,
          },
        ],
      };
      return updated;
    });
  };

  const handleChangeDirection = (x: number, y: number, d: number) => {
    setSelectedTest((prev) => {
      const directions = [Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST, Direction.SKIP];
      const new_d = directions[(directions.indexOf(d) + 1) % directions.length];

      const updatedObstacles = prev.obstacles.map((obstacle) =>
        obstacle.x === x && obstacle.y === y ? { ...obstacle, d: new_d } : obstacle
      );

      return { obstacles: updatedObstacles };
    });
  };

  const getSVGCoords = (pos: Position) => ({
    x: robotCenterX + (pos.x - robotPosition.x + 1) * cellSize,
    y: robotCenterY - (pos.y - robotPosition.y - 1) * cellSize,
  });

  const buildSmoothPath = (positions: Position[]) => {
    if (!positions || positions.length < 2) return "";
    const pts = positions.map((p) => {
      const { x, y } = getSVGCoords(p);
      return [x, y] as [number, number];
    });
    if (pts.length === 2) return `M ${pts[0][0]} ${pts[0][1]} L ${pts[1][0]} ${pts[1][1]}`;

    const d: string[] = [`M ${pts[0][0]} ${pts[0][1]}`];
    for (let i = 0; i < pts.length - 1; i++) {
      const p0 = i === 0 ? pts[0] : pts[i - 1];
      const p1 = pts[i];
      const p2 = pts[i + 1];
      const p3 = i + 2 < pts.length ? pts[i + 2] : p2;

      const c1x = p1[0] + (p2[0] - p0[0]) / 6;
      const c1y = p1[1] + (p2[1] - p0[1]) / 6;
      const c2x = p2[0] - (p3[0] - p1[0]) / 6;
      const c2y = p2[1] - (p3[1] - p1[1]) / 6;

      d.push(`C ${c1x} ${c1y}, ${c2x} ${c2y}, ${p2[0]} ${p2[1]}`);
    }
    return d.join(" ");
  };

  const ArrowAt = ({ x, y, angle, color }: { x: number; y: number; angle: number; color: string }) => {
    // nudge the arrow a bit forward along its heading so the tip is ahead of the line
    const nudge = cellSize * 0.18;
    const rad = (angle * Math.PI) / 180;
    const nx = x + Math.cos(rad) * nudge;
    const ny = y + Math.sin(rad) * nudge;

    // chevron that points toward +X when angle = 0
    const w = cellSize * 0.22; // half width
    const h = cellSize * 0.28; // arm length

    return (
      <g transform={`translate(${nx}, ${ny}) rotate(${angle})`}>
        <path
          d={`M ${-h} ${-w} L 0 0 L ${-h} ${w}`} // tip at (0,0), wings behind
          fill="none"
          stroke={color}
          strokeWidth={3}
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      </g>
    );
  };

  // Cubic segment (Catmull–Rom -> Bézier) with control points
  const segmentCubic = (p0: Position, p1: Position, p2: Position, p3: Position) => {
    const { x: x1, y: y1 } = getSVGCoords(p1);
    const { x: x2, y: y2 } = getSVGCoords(p2);
    const { x: x0, y: y0 } = getSVGCoords(p0);
    const { x: x3, y: y3 } = getSVGCoords(p3);

    const c1x = x1 + (x2 - x0) / 6;
    const c1y = y1 + (y2 - y0) / 6;
    const c2x = x2 - (x3 - x1) / 6;
    const c2y = y2 - (y3 - y1) / 6;

    return {
      d: `M ${x1} ${y1} C ${c1x} ${c1y}, ${c2x} ${c2y}, ${x2} ${y2}`,
      start: { x: x1, y: y1 },
      end: { x: x2, y: y2 },
      c2: { x: c2x, y: c2y }, // useful tangent ref
    };
  };

  // Determine if p1->p2 is reverse vs robot facing at p2 (for your colors)
  const dirToVec = (d: Direction): [number, number] =>
    d === Direction.NORTH ? [0, 1] :
    d === Direction.SOUTH ? [0,-1] :
    d === Direction.EAST  ? [1, 0] :
    d === Direction.WEST  ? [-1,0] : [0,0];

  const isReversing = (p1: Position, p2: Position) => {
    const mv: [number, number] = [p2.x - p1.x, p2.y - p1.y];
    const dv = dirToVec(p2.d);
    return mv[0] * dv[0] + mv[1] * dv[1] < 0;
  };


  return (
    <svg
      width={GRID_TOTAL_WIDTH * cellSize + offset}
      height={GRID_TOTAL_HEIGHT * cellSize + offset}
      style={{ background: THEME.bg }}
    >
      {/* X-axis labels */}
      {Array.from({ length: GRID_TOTAL_WIDTH }).map((_, col) => (
        <text
          key={`col-${col}`}
          x={(col + 1) * cellSize + offset / 2}
          y={GRID_TOTAL_HEIGHT * cellSize + offset / 2}
          textAnchor="middle"
          alignmentBaseline="middle"
          fontSize="12"
          fill={THEME.axisText}
        >
          {col}
        </text>
      ))}

      {/* Y-axis labels */}
      {Array.from({ length: GRID_TOTAL_HEIGHT }).map((_, row) => (
        <text
          key={`row-${row}`}
          x={offset / 2}
          y={row * cellSize + offset / 2}
          textAnchor="middle"
          alignmentBaseline="middle"
          fontSize="12"
          fill={THEME.axisText}
        >
          {GRID_TOTAL_HEIGHT - row - 1}
        </text>
      ))}

      {/* Grid */}
      {Array.from({ length: GRID_TOTAL_HEIGHT }).map((_, row) =>
        Array.from({ length: GRID_TOTAL_WIDTH }).map((_, col) => (
          <rect
            key={`${col}-${row}`}
            x={col * cellSize + offset}
            y={(GRID_TOTAL_HEIGHT - row - 1) * cellSize}
            width={cellSize}
            height={cellSize}
            fill="#ffffff"
            stroke={THEME.grid}
            strokeWidth={1}
            onClick={() => (canAddObstacle ? handleAddObstacle(col, row, Direction.NORTH) : undefined)}
          />
        ))
      )}

      {/* Obstacles */}
      {obstacles.map((obstacle, index) => {
        const x = obstacle.x * cellSize + offset;
        const y = (GRID_TOTAL_HEIGHT - obstacle.y - 1) * cellSize;
        const angle = getDirectionRotationAngle(obstacle.d);
        const isScanned = scannedObstacles.includes(obstacle);

        return (
          <rect
            key={index}
            x={x}
            y={y}
            width={cellSize}
            height={cellSize}
            fill={isScanned ? THEME.obstacleScanned : THEME.obstacle}
            stroke={angle !== null ? THEME.obstacleFacingEdge : undefined}
            strokeWidth={angle !== null ? 5 : 0}
            strokeDasharray={
              angle !== null ? `${cellSize}, ${GRID_TOTAL_WIDTH * cellSize}` : undefined
            } // highlight "north" edge, then rotated
            transform={
              angle !== null ? `rotate(${angle} ${x + cellSize / 2} ${y + cellSize / 2})` : undefined
            }
            onClick={() =>
              canAddObstacle ? handleChangeDirection(obstacle.x, obstacle.y, obstacle.d) : undefined
            }
          />
        );
      })}

      {/* Robot footprint area */}
      <rect
        x={robotFootprintX}
        y={robotFootprintY}
        width={cellSize * ROBOT_GRID_WIDTH}
        height={cellSize * ROBOT_GRID_HEIGHT}
        fill={THEME.robotFootprint}
        fillOpacity={0.15}
      />

      {/* Actual robot */}
      <rect
        x={robotCenterX}
        y={robotCenterY}
        width={robotWidth}
        height={robotHeight}
        fill={THEME.robotBody}
        fillOpacity={0.9}
        stroke={
          robotAngle !== null
            ? robotPosition.s === null
              ? THEME.robotNorthEdgeIdle
              : THEME.robotNorthEdgeScan
            : undefined
        }
        strokeWidth={robotAngle !== null ? 5 : 0}
        strokeDasharray={
          robotAngle !== null ? `${robotWidth}, ${GRID_TOTAL_WIDTH * cellSize}` : undefined
        }
        transform={
          robotAngle !== null
            ? `rotate(${robotAngle}  ${robotCenterX + robotWidth / 2} ${robotCenterY + robotHeight / 2})`
            : undefined
        }
      />

      {/* Smooth, segment-colored path + arrows pointing along travel */}
      {robotPath && robotPath.length > 1 && (() => {
        const segs: JSX.Element[] = [];
        const arrows: JSX.Element[] = [];
        const snaps: JSX.Element[] = [];

        // snap markers (unchanged)
        robotPath.forEach((p, i) => {
          if (p.s) {
            const { x, y } = getSVGCoords(p);
            snaps.push(
              <circle key={`snap-${i}`} cx={x} cy={y} r={cellSize / 2}
                      fill={THEME.snapMark} fillOpacity={0.35} />
            );
          }
        });

        for (let i = 1; i < robotPath.length; i++) {
          const p0 = robotPath[i - 2] ?? robotPath[i - 1];
          const p1 = robotPath[i - 1];
          const p2 = robotPath[i];
          const p3 = robotPath[i + 1] ?? robotPath[i];

          const { d, start, end, c2 } = segmentCubic(p0, p1, p2, p3);

          // Color by forward/reverse (keep your palette; add reverse colors if needed)
          const reversing = isReversing(p1, p2);
          const lineColor = reversing ? (THEME as any).reversePath ?? THEME.path : THEME.path;
          const glowColor = reversing ? (THEME as any).reverseGlow ?? THEME.path : THEME.path;

          // --- Compute *travel* direction (p1 -> p2) in SVG coords ---
          // Base heading from segment endpoints:
          const baseDx = end.x - start.x;
          const baseDy = end.y - start.y;
          let angle = Math.atan2(baseDy, baseDx) * 180 / Math.PI;

          // Safety: if cubic tangent at end is opposite, flip 180°
          const tanDx = end.x - c2.x;
          const tanDy = end.y - c2.y;
          const dot = baseDx * tanDx + baseDy * tanDy;
          if (dot < 0) angle += 180;

          // Draw the segment + arrow at the end
          segs.push(
            <g key={`seg-${i}`}>
              <path d={d} stroke={glowColor} strokeOpacity={0.30} strokeWidth={10}
                    fill="none" strokeLinecap="round" strokeLinejoin="round" />
              <path d={d} stroke={lineColor} strokeWidth={3.5}
                    fill="none" strokeOpacity={0.98}
                    strokeLinecap="round" strokeLinejoin="round" />
            </g>
          );

          arrows.push(<ArrowAt key={`arr-${i}`} x={end.x} y={end.y} angle={angle} color={lineColor} />);
        }

        return <g>{snaps}{segs}{arrows}</g>;
      })()}

    </svg>
  );
};
