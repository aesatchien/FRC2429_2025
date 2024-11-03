

FIELD SIZES - 2021

30' = 9.114 m
15' = 4.572 m

Border in non-cropped images:
0.82' = 0.25m

----------------------------------------------------------
Now, if you want a border on the field, do this in the imgui.ini:
Image is 800 x 420, field is 760x380

[GlassStorage][/SmartDashboard/Field]
image=.\sim\2021-slalom.png
top=20
left=20
bottom=400
right=780
width=9.114000
height=4.572000

BUT all of this is moot if you load the JSON file instead of the raw image.
The JSON file knows the borders are 20 pixels and it sets them for you.


