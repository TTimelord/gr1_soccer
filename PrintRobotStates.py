from GR1HW import GR1HW

bot = GR1HW("/home/gr1p24ap0039/RoCS/bin/MotorList/sources/motorlist.json")

bot.setup_robot(False)

print(bot.read())
