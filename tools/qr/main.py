import datetime
import qrcode

now = datetime.datetime.now()
now = now.strftime("%m%d%Y%H%M%S")

codetext = input('QR Code text: ')

qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr.add_data(codetext)
qr.make(fit=True)

img = qr.make_image(fill_color="black", back_color="white")
img.save("img/"+codetext+"_"+now+".png")
