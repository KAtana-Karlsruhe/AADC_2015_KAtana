In diesem Ordner sind die Sourcen der calibrations-Filter enthalten.

Folders:
*AADC_SignalCalibration:	Mit diesem Filter können Sensor- oder Aktorwerte linear skaliert werden. Dabei wird der Eingangswert  mit dem in den Eigenschaften gegebenen Wert multipliziert. 
*AADC_SignalCalibrationExt:	Dieser Filter dient zur Kalibrierung und Skalierung von Daten. Im Gegensatz zum Calibration Scaling Filter wird hier eine abschnittsweise definierte Funktion f(x) mit Stützstellen zur Interpolation der Messdaten verwendet	

Files:
* CMakeLists.txt:	Datei der CMake kette
* Readme.txt:		Diese Datei