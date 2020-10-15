# WRQR - Write and read QR-Codes

## Description
WRQR is meant to be used to write (print on paper) and read valuable protection data (e.g. private gpg keys) by means of QR codes.

## Usage
### Generate QR codes from file
Calling the following command generates the PDF file *out.pdf* with QR-codes that store the data coming from *key.asc* 
```console
$ python3 genQR.py testFiles/key.asc
```
Per default, 200 characters are stored in one QR-code of size 7cm x 7cm, 2x3 QR-codes per page. The generated QR-codes use the following pattern:
```console
$FILENAME/$CODE_NR/$MAX_CODES
$PARTIAL_TEXT_CONTENT
```




### Read QR codes from webcam
Calling the following command reads formatted and unformatted QR codes using a camera as input.
```console
$ python3 readQR.py
```
Formatted codes will automatically be combined to the original textfile.

