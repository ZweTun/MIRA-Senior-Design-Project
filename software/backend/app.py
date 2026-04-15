from unicodedata import name
import yaml
from planner.models import CellPos, GuidanceState
from planner.mosaic import Mosaic
from planner.planner import calculate_vector, make_feedback_from_diff
from transport.transport import debug_send, send_serial_packet
from vision.camera_pipeline import GRID_COLS, GRID_ROWS, CameraPipeline
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
from planner.mosaic import COLOR_TO_ID, ID_TO_COLOR, TAG_TO_COLOR_ID

# Plug in 
# Run 
# Select 
# Run 

# DONT FORGET to ENABLE
SERIAL_ENABLE = False
SERIAL_PORT = "COM11"
SERIAL_BAUD = 115200
USE_PICAMERA2 = True
CAMERA_INDEX = 1
PICAMERA_SIZE = (1296, 972)

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
]

USER_TAG = 4


current_mosaic_complete = False
grid_update = False  # reset flag after initial load
current_cell_state = []  # list of dicts with keys x, y, state (empty, unknown, correct, incorrect)
confirm_streak_by_cell = {}
unconfirm_streak_by_cell = {}
confirmation_counter = 0

INCORRECT_FRAMES = 30
CONFIRM_FRAMES = 15
UNCONFIRM_FRAMES = 7 

def handle_board_logic(color_list, list_of_cells_to_check):
    global grid, current_mosaic, current_cell_state
    global current_mosaic_complete
    global current_raw_position
    global confirm_streak_by_cell, unconfirm_streak_by_cell
    global confirmation_counter

    if color_list is None or len(color_list) != len(list_of_cells_to_check):
        current_mosaic_complete = False
        return

    new_cell_state = []
    non_empty_target_total = 0
    empty_cells_have_no_blocks = True

    for idx, cell in enumerate(list_of_cells_to_check):
        actual_color = color_list[idx]
        expected_color = grid[cell[0]][cell[1]]
        cell_key = (cell[0], cell[1])
        cell_pos = CellPos(x=cell[0], y=cell[1])
        is_target_cell = expected_color not in (None, "empty")

        if is_target_cell:
            non_empty_target_total += 1

        # Unknown 
        if actual_color == "unknown":
            new_cell_state.append({"x": cell[0], "y": cell[1], "state": "unknown"})
            continue

        # Non-target cell
        if not is_target_cell:
            confirm_streak_by_cell[cell_key] = 0
            unconfirm_streak_by_cell[cell_key] = 0
            state = "empty" if actual_color in (None, "empty") else "incorrect"
            if state == "incorrect":
                empty_cells_have_no_blocks = False
            new_cell_state.append({"x": cell[0], "y": cell[1], "state": state})
            continue

        expected_color_id = COLOR_TO_ID[expected_color]
        is_pending = cell_pos in current_mosaic.get_target_cells(expected_color_id)
        is_cell_currently_held = (
            current_raw_position is not None
            and current_raw_position.x == cell[0]
            and current_raw_position.y == cell[1]
        )


        if actual_color == expected_color:
            if is_cell_currently_held:
                # Do not confirm while the block is still actively being hovered/held on this cell.
                confirm_streak_by_cell[cell_key] = 0
                unconfirm_streak_by_cell[cell_key] = 0
                new_cell_state.append({"x": cell[0], "y": cell[1], "state": "correct"})
                continue

            confirm_streak_by_cell[cell_key] = confirm_streak_by_cell.get(cell_key, 0) + 1
            unconfirm_streak_by_cell[cell_key] = 0

            if is_pending and confirm_streak_by_cell[cell_key] >= CONFIRM_FRAMES:
                # print(f"Confirmed ({cell[0]},{cell[1]}) → {expected_color}")
                current_mosaic.remove_cell_from_color(expected_color_id, cell_pos)
                confirmation_counter += 1

            new_cell_state.append({"x": cell[0], "y": cell[1], "state": "correct"})
            continue

        # Empty/black seen 
        # Both cases treated the same 
        confirm_streak_by_cell[cell_key] = 0

        if actual_color in (None, "empty"):
            # Board surface visible — definitive absence
            unconfirm_streak_by_cell[cell_key] = unconfirm_streak_by_cell.get(cell_key, 0) + 1

            if (not is_pending) and unconfirm_streak_by_cell[cell_key] >= UNCONFIRM_FRAMES:
                # print(f"Block removed from ({cell[0]},{cell[1]}), re-adding target")
                current_mosaic.add_cell_to_color(expected_color_id, cell_pos)
                unconfirm_streak_by_cell[cell_key] = 0

            new_cell_state.append({"x": cell[0], "y": cell[1], "state": "empty"})
        else:
            # Wrong color on surface — incorrect placement
            # Wait for n frames before adding back to targets to avoid flickering 
            unconfirm_streak_by_cell[cell_key] = unconfirm_streak_by_cell.get(cell_key, 0) + 1

            if (not is_pending) and unconfirm_streak_by_cell[cell_key] >= 150:  # longer threshold for new incorrect 
                current_mosaic.add_cell_to_color(expected_color_id, cell_pos)
                # print(f"Incorrect block at ({cell[0]},{cell[1]}), expected {expected_color} but saw {actual_color} adding back to targets")
                unconfirm_streak_by_cell[cell_key] = 0
            new_cell_state.append({"x": cell[0], "y": cell[1], "state": "incorrect"})

    current_cell_state = new_cell_state
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
    unconfirm_streak_by_cell.clear()
    confirmation_counter = 0
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
                if event.kind == "offboard":
                    if event.tag_id in ID_TO_COLOR:
                        current_color = ID_TO_COLOR[event.tag_id]
                        current_position = CellPos(x=-1, y=-1)
                    # Send stop/neutral command while still keeping targets highlighted in UI.
                    packet = make_feedback_from_diff(None, CellPos(x=event.row, y=event.col))
                    if ser:
                        send_serial_packet(ser, packet)
                    continue
                
                if event.kind not in ("onboard", "placed"):
                    # Clear displayed guidance when the tracked tile is removed/offboard.
                    current_position = None
                    current_color = None
                    # Send stop signal to esp for any non-placement event
                    # print(f"Non-placement event detected, sending stop signal. Event kind: {event.kind}")
                    packet = make_feedback_from_diff(None, CellPos(x=event.row, y=event.col))
                    if ser:
                        send_serial_packet(ser, packet)
                    continue
            
                if (event.tag_id == USER_TAG):
                   
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
            # if ser and ser.in_waiting > 0:
            #     line = ser.readline().decode(errors="ignore").strip()
            #     if line:
            #         print("RX:", line)

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

# Send current block position to web client for visualization
# @app.get("/current_position")
# def get_current_position():
#     # This is a placeholder. You would replace this with the actual current position from your camera pipeline.
#     global current_position
#     global current_color
#     global current_mosaic
#     # targets = current_mosaic.get_target_cells(COLOR_TO_ID[current_color]) if current_color else None
#     # Hard coded cyan
#     # targets = current_mosaic.get_target_cells("cyan") if current_mosaic else None
#     targets = current_mosaic.get_target_cells(COLOR_TO_ID[current_color]) if current_mosaic and current_color else None
#     if current_position is None:
#         return {"current_position": None,
#                 "current_color": None,
#                 "targets": None}
#     return {"current_position": {"x": current_position.x, "y": current_position.y},
#             "current_color": current_color,
#             "targets": targets}

# Websocket endpoint to stream current position and targets to web client at 10 Hz for real-time visualization
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            global current_position, current_raw_position, current_color, current_mosaic, current_cell_state, current_mosaic_complete, board_rectified, grid, confirmation_counter
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
            }

            await websocket.send_text(json.dumps(payload))
            # 30 hz update rate for smooth visualization without overwhelming the client or network
            await asyncio.sleep(0.033)  # approximately 
    except WebSocketDisconnect:
        print("Client disconnected")

from fastapi.responses import HTMLResponse



@app.get("/status")
def status():
    return {"status": "running"}

@app.get("/", response_class=HTMLResponse)
def homepage():
    with open("index.html") as f:
        return f.read()
    