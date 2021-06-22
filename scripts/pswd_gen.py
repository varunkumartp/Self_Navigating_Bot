import pywhatkit
import random

phone_book = {
    "varun" : "+918310726474",
    "vijay" : "+919448451533" 
}

numbers = "0123456789"
symbols = "[]{()*:;<,>.?/|\~`!@#}$%^&*()_-+="
lower = "abcdefghijklmnopqrstuvwxyz"
upper = lower.upper()
length = 20
data = numbers + symbols + lower + upper

for i in range(10):
    password = "".join(random.sample(data,length))
    pywhatkit.sendwhatmsg_instantly(phone_book["vijay"], password, wait_time = 10, tab_close = True)