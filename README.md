# File Structure

## Current file structure
- rp2350_firmware
- utils/
    - angle_lib
    - calibration
    - user_interface

## Assets
- rp2350_firmware: Said's code, here we should add changes to the RP firmware
- utils: Here we keep everything we use outside of the RP
- angle_lib: Decodes RP readings and obtains angle information (serial or via wifi)
- calibration: Contains the pipeline for calibration
- user_interface: Has the user interface, visual assets