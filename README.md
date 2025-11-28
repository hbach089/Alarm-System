
# Alarm System
<p align="justify">
A FreeRTOS-based Alarm System created with STM32 Nucleo board using the F446RE Arm Cortex-M4 32-bit RISC core. The goal of this project is to provide home owners with a     reliable security solution. After arming the system using a password, the alarm is triggered 60 seconds after detecting any movement. The user can then disable this feature by entering the same password to disarm the system. The user has the option to change the password by storing a new password in Flash memory. 
</p>

## Table of Contents
- [Hardware Setup](https://github.com/hbach089/Alarm-System?tab=readme-ov-file#hardware-setup)
- [RTOS Configuration](https://github.com/hbach089/Alarm-System?tab=readme-ov-file#rtos-configuration)
- [How to run](https://github.com/hbach089/Alarm-System?tab=readme-ov-file#how-to-run)
- [Live Demo](https://github.com/hbach089/Alarm-System?tab=readme-ov-file#live-demo)

## Hardware Setup
Additional Hardware Components used: 
- 220 Ω Resistors (x2)
- 100 Ω Resistor (x1)
- Red LED & Green LED (x1 each)
- PIR motion sensor (x1) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b><i>For more information, consult [this document](https://cdn-learn.adafruit.com/downloads/pdf/pir-passive-infrared-proximity-motion-sensor.pdf)</i></b>
- Passive Buzzer (x1)
- 3x4 keypad matrix (x1)  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<b><i>For more information, consult [this website](google.com/search?q=keypad+membrane+stm32&rlz=1C1GCEA_enCA1168CA1168&oq=keypad+mem&gs_lcrp=EgZjaHJvbWUqBggAEEUYOzIGCAAQRRg7MgcIARAAGIAEMgcIAhAAGIAEMgYIAxBFGEAyBggEEEUYOTIHCAUQABiABDIHCAYQABiABDIICAcQABgWGB7SAQg0Mzk0ajBqNKgCALACAQ&sourceid=chrome&ie=UTF-8#sv=CBAS_xcKvBcKBtrZ29IPABKxFwr0BQrxBbrZ29IP6gUKNmh0dHBzOi8vY29udHJvbGxlcnN0ZWNoLmNvbS91c2UtNHg0LWtleXBhZC13aXRoLXN0bTMyLxIPQ29udHJvbGxlcnNUZWNoGpoFZGF0YTppbWFnZS9wbmc7YmFzZTY0LGlWQk9SdzBLR2dvQUFBQU5TVWhFVWdBQUFCd0FBQUFjQ0FNQUFBQkYweSttQUFBQVRsQk1WRVZIY0V6MWhoSDJoeEwyaHhMMmh4TDJoeEwyaHhMMmh4TDJoeEwyaHhMMmh4TDJoeEwyaHhMMWhoTDJoeElVVUhjUVVIZ09VSGdPVUhnT1VIZ09VSGdOVUhrUFVIZ05VSGtOVUhrTlVIbnR3SGVPQUFBQUduUlNUbE1BS0l1dnZ6bE9mWm5ULytwY0dHd1lLcEZwUkh2RVZxemIvUDl6N1gwQUFBRW9TVVJCVkhnQnJZNkhpb1V3RkVRbnpSaGZUQy82L3orNk41WU55OUo1aHd1Q2g1a0p2Z1BqUWdncHVmcXZGcWtYTGxaaHR1MWovNnJkdUYxd3l2SlZma2p2bUZnSkxlaDJKNTBlZHB2ZG5FTmFHTFVieGFSbVkyTmpqeE1rZG1iZ0JOWVZOK3J6dUFVR1NtTFZNTE5OTFdOTzJNZU5QQ2FNYm5HalU0SzcyL21BQ1hlU3dWaTlQcm1ZeUhya1VpNDdKclZ5NHQ2cjBhT2sxbkljbXhxU0lvcGJESHErMHJubSt6MldBMWpXb1NpVGdkQVI4OVc1T01veUxqRm9zWVNDQStrYTFGeGhsTm9yR0NwNmFRZHlMeEVUdHQrdUZGUlAzNVk4RUY0N1hmQlUyeHVBTkcycmlLOTdTcy9YOW9qYWNJek8zOEZ3OWd5RVVuMGVSNFVqLzVMUDh6eDZLYldGbWpBdVl4SU9zalhHWGtMc0tCVi9TUlN1bld5T016YWh2N1gzWGp5K3dnOTNaaEdFYVRTYTNBQUFBQUJKUlU1RXJrSmdnZz09IAE4AQqfAgqcAtLZ29IPlQIKNkludGVyZmFjZSA0w5c0IEtleXBhZCB3aXRoIFNUTTMyICYgRGlzcGxheSBLZXkgUHJlc3MgfBI2aHR0cHM6Ly9jb250cm9sbGVyc3RlY2guY29tL3VzZS00eDQta2V5cGFkLXdpdGgtc3RtMzIvGqIBSW4gdGhpcyB0dXRvcmlhbCB3ZSB3aWxsIGludGVyZmFjZSBhIDTDlzQga2V5cGFkIHdpdGggU1RNMzIuIEkgYW0gZ29pbmcgdG8gdXNlIFNUTTMyRjEwM0M4IG1pY3JvY29udHJvbGxlciBhbmQgdGhlIGtleXBhZCBpcyBhIG1lbWJyYW5lIHN3aXRjaCA0w5c0IG1hdHJpeCBrZXlwYWQuCtsICtgIwtnb0g_RCBI2SW50ZXJmYWNlIDTDlzQgS2V5cGFkIHdpdGggU1RNMzIgJiBEaXNwbGF5IEtleSBQcmVzcyB8GoMDCjZodHRwczovL2NvbnRyb2xsZXJzdGVjaC5jb20vdXNlLTR4NC1rZXlwYWQtd2l0aC1zdG0zMi8SNkludGVyZmFjZSA0w5c0IEtleXBhZCB3aXRoIFNUTTMyICYgRGlzcGxheSBLZXkgUHJlc3MgfBqiAUluIHRoaXMgdHV0b3JpYWwgd2Ugd2lsbCBpbnRlcmZhY2UgYSA0w5c0IGtleXBhZCB3aXRoIFNUTTMyLiBJIGFtIGdvaW5nIHRvIHVzZSBTVE0zMkYxMDNDOCBtaWNyb2NvbnRyb2xsZXIgYW5kIHRoZSBrZXlwYWQgaXMgYSBtZW1icmFuZSBzd2l0Y2ggNMOXNCBtYXRyaXgga2V5cGFkLiABKgJlbjICQ0FCHy9zP3RibT1tYXAmZ3Nfcmk9bWFwcyZzdWdnZXN0PXBKMEFJa0VIb29QVm9xWVd2dHpxa3R5ZTg3ZTRMLXU0R25rYVE6MTc2NDM0ODEyOTI2NlIPQ29udHJvbGxlcnNUZWNoUnIYBSo2SW50ZXJmYWNlIDTDlzQgS2V5cGFkIHdpdGggU1RNMzIgJiBEaXNwbGF5IEtleSBQcmVzcyB8OjZodHRwczovL2NvbnRyb2xsZXJzdGVjaC5jb20vdXNlLTR4NC1rZXlwYWQtd2l0aC1zdG0zMi9yUwo2aHR0cHM6Ly9jb250cm9sbGVyc3RlY2guY29tL3VzZS00eDQta2V5cGFkLXdpdGgtc3RtMzIvKCMyF1NvdXJjZTogQ29udHJvbGxlcnNUZWNoescDGjZJbnRlcmZhY2UgNMOXNCBLZXlwYWQgd2l0aCBTVE0zMiAmIERpc3BsYXkgS2V5IFByZXNzIHwiFndlYnJlc3VsdF9ZdWZsS3pKMTZLY0owAWDujwZqCk1TQUYgLSBTUlBy1gKqARBXRUJfUkVTVUxUX0lOTkVSsgFCCjZodHRwczovL2NvbnRyb2xsZXJzdGVjaC5jb20vdXNlLTR4NC1rZXlwYWQtd2l0aC1zdG0zMi8SBEJMVVIYACAA4gH6AQqpAUluIHRoaXMgdHV0b3JpYWwgd2Ugd2lsbCBpbnRlcmZhY2UgYSA0w5c0IGtleXBhZCB3aXRoIFNUTTMyLiBJIGFtIGdvaW5nIHRvIHVzZSA8Yj5TVE0zMkYxMDNDOCBtaWNyb2NvbnRyb2xsZXI8L2I-IGFuZCB0aGUga2V5cGFkIGlzIGEgbWVtYnJhbmUgc3dpdGNoIDTDlzQgbWF0cml4IGtleXBhZC4SSEludGVyZmFjZSA0w5c0IDxiPktleXBhZDwvYj4gd2l0aCA8Yj5TVE0zMjwvYj4gJmFtcDsgRGlzcGxheSBLZXkgUHJlc3MgfBgAIBqgAWLAAQHgAQHoAQEKtwYKtAbK2dvSD60GEogGIoUGL3NlYXJjaC9hYm91dC10aGlzLXJlc3VsdD9vcmlnaW49d3d3Lmdvb2dsZS5jb20mcmVxPUNqWm9kSFJ3Y3pvdkwyTnZiblJ5YjJ4c1pYSnpkR1ZqYUM1amIyMHZkWE5sTFRSNE5DMXJaWGx3WVdRdGQybDBhQzF6ZEcwek1pOFNCQm9DQ0FBYXhnTVNBZ2dBR2dBaUFDb0FNZ1lJQWhJQ1kyRTZBRUlFQ0FFUUFFb0FXZ0J5QUhvQWdrQ2dBd2dBRUFBWUFDQUFLaU1LQm10bGVYQmhaQlh1YzR4QUdnZHJaWGx3WVdSekdndHBkanR3TzJ0bGVYQmhaQ29wQ2dodFpXMWljbUZ1WlJVVzloSV9HZ2x0WlcxaWNtRnVaWE1hRFdsMk8zQTdiV1Z0WW5KaGJtVXFHQW9GYzNSdE16SVZ1TkdCUUJvS2FYWTdjRHR6ZEcwek1qSTJhSFIwY0hNNkx5OWpiMjUwY205c2JHVnljM1JsWTJndVkyOXRMM1Z6WlMwMGVEUXRhMlY1Y0dGa0xYZHBkR2d0YzNSdE16SXZPa2hKYm5SbGNtWmhZMlVnTk1PWE5DQThZajVMWlhsd1lXUThMMkktSUhkcGRHZ2dQR0ktVTFSTk16SThMMkktSUNaaGJYQTdJRVJwYzNCc1lYa2dTMlY1SUZCeVpYTnpJSHhDcVFGSmJpQjBhR2x6SUhSMWRHOXlhV0ZzSUhkbElIZHBiR3dnYVc1MFpYSm1ZV05sSUdFZ05NT1hOQ0JyWlhsd1lXUWdkMmwwYUNCVFZFMHpNaTRnU1NCaGJTQm5iMmx1WnlCMGJ5QjFjMlVnUEdJLVUxUk5NekpHTVRBelF6Z2diV2xqY205amIyNTBjbTlzYkdWeVBDOWlQaUJoYm1RZ2RHaGxJR3RsZVhCaFpDQnBjeUJoSUcxbGJXSnlZVzVsSUhOM2FYUmphQ0EwdzVjMElHMWhkSEpwZUNCclpYbHdZV1F1SWdJUUFVZ0FXQUJvQUEmaGw9ZW4tQ0EmZ2w9Q0EaFmh0dHBzOi8vd3d3Lmdvb2dsZS5jb21aAGABaAFwAHgAEj5hdHJpdGVtLWh0dHBzOi8vY29udHJvbGxlcnN0ZWNoLmNvbS91c2UtNHg0LWtleXBhZC13aXRoLXN0bTMyLxgvIPTYinswAg)</i></b>
- 16x2 I2C LCD (x1)
- Jumper Wires
  
## RTOS Configuration
### FreeRTOS version
CMSIS FreeRTOS V2 was used for this project. For further information, please consult [this document](https://arm-software.github.io/CMSIS_5/RTOS2/html/index.html)

### Brief description of responsibilities
- Each hardware component used in this project has a/multiple dedicated task(s) responsible for managing it. As this is a multithreaded environment, task notifications were utilized to allow tasks to communicate with each other.
- Mutexes were used to protect shared resources such as global variables and the LCD screen when accessing them.

### Tasks summary
- <b>StartKeyPadIptTask: </b>Task responsible for handling the presses on the Keypad component.
- <b>StartLCDLine2PwordTask:</b> Task responsible for displaying the hidden password on the second line of the LCD, as a key is pressed on the Keypad.
- <b>StartRED_LEDTask:</b> Task responsible for turning the Red LED on and off, depending on the state of the system.
- <b>StartGreen_LEDTask:</b> Task responsible for turning the Green LED on and off, depending on the state of the system.
- <b>StartLCDLine1Task:</b> Task responsible for displaying the state of the system (Armed or Disarmed), on the first line of the LCD.
- <b>StartResetPwordTask:</b> Task responsible for displaying the state of the system when the user wants to reset the password (Old password followed by New password). The New Password state can only be accessed once the old password is confirmed to match the password stored in Flash memory. 
- <b>StartPIRsensorTask:</b> Task responsible for handling the PIR sensor and Buzzer logic. A PWM signal is used to control the Buzzer.
- <b>StartLCDLine2TIMTask:</b> Task that displays the countdown of 60 seconds on the second line of the LCD when movement is detected. The task will only be used if the system is armed. Once disarmed, the countdown is erased.

## How to Run
- Clone this project
  ```
  git clone https://github.com/hbach089/Alarm-System.git
  ```
- Install the latest version of [CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
- Ensure that no important data is present Flash memory.
  - Flash memory is divided into sectors.
  - To write to Flash, we must erase a sector, and then write to it (i.e overwrite the existing data).
  - For more information on Flash, consult the [reference manual's section on Flash](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=64).
- System clock speed must be set to 48 Mhz. 
- Open the project within CubeIDE, then compile and run.

## Live Demo

<i> Turn audio on! </i>
### Program flow
1) User enters password; alarm is armed.
2) PIR sensor detects movement; system starts countdown.
3) After 60 seconds (shortened to 10 in video), buzzer sounds the alarm.
4) User disarms the alarm, and PIR sensor no longer detects movement.
5) User changes password; program goes back step 1.
<div align="center">
  <video src="https://github.com/user-attachments/assets/9c63a6ca-0221-4a1b-9a11-bf6e79b6c3f6" width="400"/>
</div>

