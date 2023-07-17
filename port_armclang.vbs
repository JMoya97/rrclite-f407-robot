Set WshShell = CreateObject("WScript.Shell")
source_path = ".\Middlewares\Third_Party\FreeRTOS\Source\portable\GCC"
destination_path = ".\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS"

WshShell.Run "cmd /c xcopy /E /I /Y """ & source_path & """ """ & destination_path & """", 0, True

Set WshShell = Nothing