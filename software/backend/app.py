from unicodedata import name
import yaml
from pathlib import Path
from planner.models import CellPos, GuidanceState
from planner.mosaic import Mosaic
from planner.planner import calculate_vector, make_feedback_from_diff
from transport.transport import debug_send, send_serial_packet
from vision.camera_pipeline import GRID_COLS, GRID_ROWS, CameraPipeline, WRIST_TAG_CONFIGS
import serial


import cv2

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import asyncio
import json

from pydantic import BaseModel
import threading
import time
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from planner.mosaic import COLOR_TO_ID, ID_TO_COLOR, TAG_TO_COLOR_ID

# Plug in 
# Run 
# Select 
# Run 

# DONT FORGET to ENABLE
SERIAL_ENABLE = True
SERIAL_PORT = "COM11"
SERIAL_BAUD = 115200
USE_PICAMERA2 = False
CAMERA_INDEX = 1
PICAMERA_SIZE = (1296, 972)
PICAMERA_FRAME_ORDER = "rgb"

ser = None
my_name = ""
current_position = None
current_raw_position = None
current_color = None
current_mosaic = None
current_cell_state = None
current_mosaic_complete = False
board_rectified = False
grid = None
grid_update = False

if SERIAL_ENABLE:
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except serial.SerialException as e:
        print("Serial open failed:", e)
        ser = None



TEST_MOSAIC = [
    ["green",   "green",   "green",   "green",   "green",   "green",   "green",   "green"],
    ["green",   "green",   "green",   "green",   "green",   "green",   "green",   "green"],
    ["cyan",    "cyan",    "cyan",    "cyan",    "cyan",    "cyan",    "cyan",    "cyan"],
    ["magenta", "magenta", "magenta", "magenta", "red",     "magenta", "magenta", "magenta"],
    ["green",   "green",   "green",   "green",   "green",   "green",   "green",   "green"],
    ["green",   "green",   "green",   "green",   "green",   "green",   "green",   "green"],
    ["cyan",    "cyan",    "cyan",    "cyan",    "cyan",    "cyan",    "cyan",    "cyan"],
    ["magenta", "magenta", "magenta", "magenta", "magenta", "magenta", "magenta", "magenta"],
]


def build_mosaic_from_grid(grid):
    mosaic = Mosaic()
    for row in range(len(grid)):
        for col in range(len(grid[row])):
            color_id = COLOR_TO_ID[grid[row][col]]
            mosaic.add_cell_to_color(color_id, CellPos(x=row, y=col))
    return mosaic

def destroy_mosaic(mosaic):
    if mosaic:
        mosaic.targets_by_color.clear()

def ack_test():
    ser.write(b"1.0,1,0,2,3,4\n")

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("RX:", line)
def load_mosaic_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)["grid"]


LIBRARY_MOSAICS = [
    {"endpoint": "mosaic1", "name": "flower", "path": "backend/yaml_assets/flower.yaml"},
    {"endpoint": "mosaic2", "name": "boat", "path": "backend/yaml_assets/boat.yaml"},
    {"endpoint": "mosaic3", "name": "moon", "path": "backend/yaml_assets/moon.yaml"},
    {"endpoint": "mosaic4", "name": "smiley", "path": "backend/yaml_assets/smiley.yaml"},
    {"endpoint": "mosaic5", "name": "sad", "path": "backend/yaml_assets/sad.yaml"},
]


def build_empty_grid(rows=GRID_ROWS, cols=GRID_COLS):
    return [["empty" for _ in range(cols)] for _ in range(rows)]

WRIST_TAGS = set(WRIST_TAG_CONFIGS)
FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"



current_mosaic_complete = False
grid_update = False  # reset flag after initial load
current_cell_state = []  # list of dicts with keys x, y, state (empty, unknown, current, correct, incorrect)
confirm_streak_by_cell = {}
incorrect_streak_by_cell = {}
confirmation_counter = 0
current_blocks_onboard = 0
total_non_empty_expected = 0

INCORRECT_FRAMES = 20
CURRENT_FRAMES = 10
CONFIRM_FRAMES = 10
HOLD_CONFIRM_BUFFER_RADIUS = 1  # 3x3 area centered on held cell

def handle_board_logic(color_list, list_of_cells_to_check):
    global grid, current_mosaic, current_cell_state
    global current_mosaic_complete
    global current_raw_position
    global confirm_streak_by_cell, incorrect_streak_by_cell
    global confirmation_counter
    global current_blocks_onboard
    global total_non_empty_expected

    if color_list is None or len(color_list) != len(list_of_cells_to_check):
        current_blocks_onboard = 0
        total_non_empty_expected = 0
        current_mosaic_complete = False
        return

    new_cell_state = []
    non_empty_target_total = 0
    empty_cells_have_no_blocks = True
    blocks_onboard = 0

    def add_state(row, col, state):
        new_cell_state.append({"x": row, "y": col, "state": state})

    def bump(counter, key):
        counter[key] = counter.get(key, 0) + 1
        return counter[key]

    def is_empty_reading(color):
        return color in (None, "empty")

    for idx, (row, col) in enumerate(list_of_cells_to_check):
        actual_color = color_list[idx]
        expected_color = grid[row][col]
        cell_key = (row, col)
        cell_pos = CellPos(x=row, y=col)
        is_target_cell = expected_color not in (None, "empty")

        # Count currently visible onboard blocks irrespective of correctness.
        if actual_color not in (None, "empty", "unknown"):
            blocks_onboard += 1

        if is_target_cell:
            non_empty_target_total += 1

        # Ignore unknown readings entirely for board-logic decisions.
        # Do not compare to expected color and do not update any streak counters.
        if actual_color == "unknown":
            if is_target_cell:
                expected_color_id = COLOR_TO_ID[expected_color]
                is_pending = cell_pos in current_mosaic.get_target_cells(expected_color_id)
                add_state(row, col, "correct" if not is_pending else "unknown")
            else:
                add_state(row, col, "empty")
            continue

        if not is_target_cell:
            confirm_streak_by_cell[cell_key] = 0

            if is_empty_reading(actual_color):
                incorrect_streak_by_cell[cell_key] = 0
                state = "empty"
            else:
                wrong_frames = bump(incorrect_streak_by_cell, cell_key)
                if wrong_frames >= INCORRECT_FRAMES:
                    state = "incorrect"
                elif wrong_frames >= CURRENT_FRAMES:
                    # Stable non-empty presence, but not long enough to mark as incorrect.
                    state = "current"
                else:
                    # Short detections are treated as unknown to avoid hand-motion false flashes.
                    state = "unknown"

            if state == "incorrect":
                empty_cells_have_no_blocks = False
            add_state(row, col, state)
            continue

        expected_color_id = COLOR_TO_ID[expected_color]
        is_pending = cell_pos in current_mosaic.get_target_cells(expected_color_id)
        is_cell_currently_held = (
            current_raw_position is not None
            and current_raw_position.x == row
            and current_raw_position.y == col
        )
        in_hold_confirm_buffer = (
            current_raw_position is not None
            and abs(current_raw_position.x - row) <= HOLD_CONFIRM_BUFFER_RADIUS
            and abs(current_raw_position.y - col) <= HOLD_CONFIRM_BUFFER_RADIUS
        )

        if actual_color == expected_color:
            incorrect_streak_by_cell[cell_key] = 0
            if is_pending and in_hold_confirm_buffer:
                # Block confirmations in a 3x3 neighborhood around the held cell.
                confirm_streak_by_cell[cell_key] = 0
                add_state(row, col, "unknown")
                continue

            if is_cell_currently_held:
                # Do not confirm while the block is still actively being hovered/held on this cell.
                confirm_streak_by_cell[cell_key] = 0
                add_state(row, col, "correct" if not is_pending else "unknown")
                continue

            confirm_frames = bump(confirm_streak_by_cell, cell_key)

            if is_pending and confirm_frames >= CONFIRM_FRAMES:
                current_mosaic.remove_cell_from_color(expected_color_id, cell_pos)
                confirmation_counter += 1
                add_state(row, col, "correct")
                continue

            if is_pending and confirm_frames >= CURRENT_FRAMES:
                # Pending correct placement seen long enough to be considered current.
                add_state(row, col, "current")
                continue

            add_state(row, col, "correct" if not is_pending else "unknown")
            continue

        # Empty/black seen; both cases treated the same for confirm streak reset.
        confirm_streak_by_cell[cell_key] = 0

        if is_empty_reading(actual_color):
            incorrect_streak_by_cell[cell_key] = 0
            # Board surface visible — definitive absence.
            if not is_pending:
                current_mosaic.add_cell_to_color(expected_color_id, cell_pos)

            add_state(row, col, "empty")
            continue

        # Wrong color on surface.
        if not is_pending:
            # Once confirmed, ignore non-empty detections (e.g., hands/occlusions).
            # Only an empty board reading should re-add this target.
            incorrect_streak_by_cell[cell_key] = 0
            add_state(row, col, "correct")
            continue

        if is_cell_currently_held:
            incorrect_streak_by_cell[cell_key] = 0
            add_state(row, col, "unknown")
            continue

        incorrect_frames = bump(incorrect_streak_by_cell, cell_key)
        if incorrect_frames < INCORRECT_FRAMES:
            # For already-confirmed cells, keep visual correctness until incorrect is confirmed.
            add_state(row, col, "correct" if not is_pending else "unknown")
            continue

        # Confirmed incorrect after INCORRECT_FRAMES to avoid hand-motion false positives.
        add_state(row, col, "incorrect")

    current_cell_state = new_cell_state
    current_blocks_onboard = blocks_onboard
    total_non_empty_expected = non_empty_target_total
    pending = sum(
        len(cells)
        for color_id, cells in current_mosaic.targets_by_color.items()
        if color_id != COLOR_TO_ID.get("empty")
    )
    # Print mosiac targets
    # print("Current mosaic targets:")
    # for color_id, cells in current_mosaic.targets_by_color.items():
    #     if color_id != COLOR_TO_ID.get("empty"):
    #         print(f"  {ID_TO_COLOR.get(color_id)}: {len(cells)} cells")
    current_mosaic_complete = (
        non_empty_target_total > 0
        and pending == 0
        and empty_cells_have_no_blocks
    )

def handle_grid_update(camera):
    global grid
    global grid_update
    global current_mosaic
    global current_mosaic_complete
    global current_cell_state
    global current_position
    global current_raw_position
    global current_color
    global confirmation_counter
    global board_rectified
    global incorrect_streak_by_cell
    global current_blocks_onboard
    global total_non_empty_expected
    print("Mosaic selection changed, rebuilding mosaic...")
    # Destroy old mosaic to clear targets and free resources
    destroy_mosaic(current_mosaic)
    current_mosaic = build_mosaic_from_grid(grid)
    camera.update_grid(grid)
    current_mosaic_complete = False
    current_cell_state = []
    current_position = None
    current_raw_position = None
    current_color = None
    board_rectified = False
    confirm_streak_by_cell.clear()
    incorrect_streak_by_cell.clear()
    confirmation_counter = 0
    current_blocks_onboard = 0
    total_non_empty_expected = 0
    grid_update = False

def main():
    global current_position
    global current_raw_position
    global current_color 
    global current_mosaic
    global current_cell_state
    global current_mosaic_complete
    global board_rectified
    global grid
    global grid_update 
    global current_blocks_onboard
    
    while not grid_update:
        print("Waiting for initial mosaic selection...")
        time.sleep(1)
    destroy_mosaic(current_mosaic)  # Clear any existing mosaic data
    current_mosaic = build_mosaic_from_grid(grid)
    camera = CameraPipeline(
        grid=grid,
        camera_index=CAMERA_INDEX,
        use_picamera2=USE_PICAMERA2,
        picamera_size=PICAMERA_SIZE,
        picamera_frame_order=PICAMERA_FRAME_ORDER,
    )
    all_cells = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            all_cells.append((row, col))
    try:
        while True:
            if grid_update:
                handle_grid_update(camera)

         
            
            list_of_cells_to_check = all_cells.copy()  # check all cells for updates each frame

  
            # print("Cells:", list_of_cells_to_check)
            _, events, should_quit, color_list = camera.step(cells_to_check=(list_of_cells_to_check))
            board_rectified = camera.Hmat is not None
            current_raw_position = (
                CellPos(x=camera.raw_block_cell[0], y=camera.raw_block_cell[1])
                if camera.raw_block_cell is not None
                else None
            )
            # Grid update detected, rebuild mosaic
            handle_board_logic(color_list, list_of_cells_to_check)

            # print(f"Detected {len(events)} tile events")

            for event in events:
                # print(f"EVENT: tag={event.tag_id}, kind={event.kind}, row={event.row}, col={event.col}")
                block_still_held = current_raw_position is not None
                if event.kind == "offboard":
                    if not block_still_held and event.tag_id in ID_TO_COLOR:
                        current_color = ID_TO_COLOR[event.tag_id]
                        current_position = CellPos(x=-1, y=-1)
                    # Send stop/neutral command while still keeping targets highlighted in UI.
                    packet = make_feedback_from_diff(None, CellPos(x=event.row, y=event.col))
                    if ser:
                        send_serial_packet(ser, packet)
                    continue
                
                if event.kind not in ("onboard", "placed"):
                    # Only clear displayed guidance when the held block is truly gone.
                    if not block_still_held:
                        current_position = None
                        current_color = None
                    # Send stop signal to esp for any non-placement event
                    # print(f"Non-placement event detected, sending stop signal. Event kind: {event.kind}")
                    packet = make_feedback_from_diff(None, CellPos(x=event.row, y=event.col))
                    if ser:
                        send_serial_packet(ser, packet)
                    continue
            
                if event.tag_id in WRIST_TAGS:
                   
                    packet = make_feedback_from_diff(None, CellPos(x=event.row, y=event.col))
                    if ser:
                        send_serial_packet(ser, packet)
                    continue

                if event.tag_id not in TAG_TO_COLOR_ID:
                    # print(f"Unknown color for tag {event.tag_id}")
                    continue



                color_id = TAG_TO_COLOR_ID[event.tag_id]
                color_name = ID_TO_COLOR[color_id]
                current_color = color_name
                current_position = CellPos(x=event.row, y=event.col)

                targets = current_mosaic.get_target_cells(color_id)

                # print(f"Current color: {color_name}, position: ({current_position.x},{current_position.y}), targets: {[f'({t.x},{t.y})' for t in targets]}")
                # print(f"Target list for color {color_name} (id {color_id}): {[f'({t.x},{t.y})' for t in targets]}")
                diff = calculate_vector(targets, current_position)

                
                # if event.kind == "placed":
                #     in_bounds = 0 <= event.row < GRID_ROWS and 0 <= event.col < GRID_COLS
                #     expected_color_id = COLOR_TO_ID.get(grid[event.row][event.col]) if in_bounds else None
                #     if expected_color_id is not None and color_id == expected_color_id:
                #         # Target removal is handled by board scan after stable confirmation.
                #         print(f"Tile placed correctly at ({event.row}, {event.col})")
                #     else:
                #         if expected_color_id is not None:
                #             current_mosaic.add_cell_to_color(
                #                 expected_color_id,
                #                 CellPos(x=event.row, y=event.col),
                #             )
                #         # packet = make_feedback_from_diff(CellPos(0, 0), current_position, state=GuidanceState.INCORRECT)
                #         print(f"Tile placed incorrectly at ({event.row}, {event.col})")
                
                packet = make_feedback_from_diff(diff, current_position)

                # print(
                #     f"Tile color={color_name}, "
                #     f"current=({current_position.x},{current_position.y}), "
                #     f"diff={None if diff is None else (diff.x, diff.y)}, "
                #     f"state={event.kind}, "
                # )
                # debug_send(packet)
                if ser:
                    send_serial_packet(ser, packet)

            if should_quit:
                break
            
            # ESP DEBUG
            if ser and ser.in_waiting > 0:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print("RX:", line)

            # if my_name:
            #     print(f"Current mosaic: {my_name}", flush=True)
                
   


    finally:
        camera.close()
    # print("Main loop exited")


# if __name__ == "__main__":
#     main()







class NameRequest(BaseModel):
    name: str


def background_loop():
    global my_name
    print("Background loop started")
    i = 0
    while True:
        print(f"Running background loop {i}, mosaic={my_name}", flush=True)
        time.sleep(1)
        i += 1




@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Starting background thread")
    # thread = threading.Thread(target=background_loop, daemon=True)
    # thread.start()
    # yield

    thread = threading.Thread(target=main, daemon=True)
    thread.start()
    yield
    print("Lifespan ending, background thread should exit")

    

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)



@app.post("/greet")
def greet_user(req: NameRequest):
    global my_name
    my_name = req.name  
    print(f"Greeting user: {req.name}")
    return {"message": f"Hello, {req.name}!"}


@app.post("/mosaic1")
def set_mosaic1(req: NameRequest):
    global my_name
    global grid
    global grid_update
    my_name = req.name
    grid = load_mosaic_yaml("backend/yaml_assets/flower.yaml")
    print(f"Setting mosaic1 for user: {req.name}")
    grid_update = True
    return {"message": f"Selected Mosaic: {req.name}!",
            "grid": grid}

@app.post("/mosaic2")
def set_mosaic2(req: NameRequest):
    global my_name
    global grid
    global grid_update
    my_name = req.name
    print(f"Setting mosaic2 for user: {req.name}")
    grid = load_mosaic_yaml("backend/yaml_assets/boat.yaml")
    grid_update = True
    return {"message": f"Selected Mosaic: {req.name}!",
            "grid": grid}


@app.post("/mosaic3")
def set_mosaic3(req: NameRequest):
    global my_name
    global grid
    global grid_update
    my_name = req.name
    print(f"Setting mosaic3 for user: {req.name}")
    grid = load_mosaic_yaml("backend/yaml_assets/moon.yaml")
    grid_update = True
    return {"message": f"Selected Mosaic: {req.name}!",
            "grid": grid}

@app.post("/mosaic4")
def set_mosaic4(req: NameRequest):
    global my_name
    global grid
    global grid_update
    my_name = req.name
    print(f"Setting mosaic4 for user: {req.name}")
    grid = load_mosaic_yaml("backend/yaml_assets/smiley.yaml")
    grid_update = True
    return {"message": f"Selected Mosaic: {req.name}!",
            "grid": grid}

@app.post("/mosaic5")
def set_mosaic5(req: NameRequest):
    global my_name
    global grid
    global grid_update
    my_name = req.name
    print(f"Setting mosaic5 for user: {req.name}")
    grid = load_mosaic_yaml("backend/yaml_assets/sad.yaml")
    grid_update = True
    return {"message": f"Selected Mosaic: {req.name}!",
            "grid": grid}


@app.post("/mosaic_empty")
def set_mosaic_empty(req: NameRequest):
    global my_name
    global grid
    global grid_update
    my_name = req.name
    print(f"Setting empty mosaic for user: {req.name}")
    grid = build_empty_grid()
    grid_update = True
    return {
        "message": f"Selected Mosaic: {req.name}!",
        "grid": grid,
    }


@app.get("/mosaic_library")
def get_mosaic_library():
    mosaics = []
    for item in LIBRARY_MOSAICS:
        mosaics.append(
            {
                "endpoint": item["endpoint"],
                "name": item["name"],
                "grid": load_mosaic_yaml(item["path"]),
            }
        )
    return {"mosaics": mosaics}

# Websocket endpoint to stream current position and targets to web client at 10 Hz for real-time visualization
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            global current_position, current_raw_position, current_color, current_mosaic, current_cell_state, current_mosaic_complete, board_rectified, grid, confirmation_counter, current_blocks_onboard, total_non_empty_expected
            # print(current_cell_state)
            targets_raw = (
                current_mosaic.get_target_cells(COLOR_TO_ID[current_color])
                if current_mosaic and current_color
                else None
            )

            # Convert CellPos objects to dicts for JSON serialization
            if targets_raw is not None:
                targets = [{"x": t.x, "y": t.y} for t in targets_raw]
            else:
                targets = None

            payload = {
                "current_position": (
                    {"x": current_position.x, "y": current_position.y}
                    if current_position else None
                ),
                "raw_position": (
                    {"x": current_raw_position.x, "y": current_raw_position.y}
                    if current_raw_position else None
                ),
                "current_color": current_color,
                "targets": targets,
                "cell_state": current_cell_state,
                "mosaic_complete": current_mosaic_complete,
                "board_rectified": board_rectified,
                "grid": grid,
                "confirmation_counter": confirmation_counter,
                "current_blocks_onboard": current_blocks_onboard,
                "total_non_empty_expected": total_non_empty_expected,
            }

            await websocket.send_text(json.dumps(payload))
            # 30 hz update rate for smooth visualization without overwhelming the client or network
            await asyncio.sleep(0.033)  # approximately 30 Hz
    except WebSocketDisconnect:
        print("Client disconnected")

from fastapi.responses import HTMLResponse



@app.get("/status")
def status():
    return {"status": "running"}

@app.get("/", response_class=HTMLResponse)
def homepage():
    with open(FRONTEND_DIR / "index.html", encoding="utf-8") as f:
        return f.read()


app.mount("/frontend", StaticFiles(directory=str(FRONTEND_DIR), html=True), name="frontend")
    