# Modular print in place 35mm film scanner writeup

I've been building a 35mm film scanner using Lego, and more recently 3D printing.

Scanning film is surprisingly difficult and there are plenty of not so obvious pitfalls. Finding these out went roughly like:

- First just try sliding the negatives between two rails holding them by the edges, but they would often curl up too much and cause distortion and out of focus problems.
- So just force them between two sheets of glass right? Good idea, but now your scans will randomly contain newton rings (this example is quite subtle): 
  
  ![image](https://user-images.githubusercontent.com/3673134/180775636-2f7b4b28-7bea-44c2-bad7-1c737690ed8a.png)
- So maybe just one sheet of glass behind the negative? Seemed to work at first, but then occasionally more netwon rings pop up for film that hasn't curled as much.
- Back to using no glass sheets but making the dimensions of the rails very tight, which was pretty difficult to accomplish with Lego. Use high aperture values to get rid of focus problems, but oh... now the plane of focus is wide enough that patterns on my backlight are starting to show through in the images.
- So get a [plastic diffuser sheet](https://www.amazon.de/-/en/gp/product/B089SPZTGY) and place it in front of my backlight. I think we're good now

## Prototypes

This project has gone through various crazy prototypes in Lego, the last of which actually ended up scanning thousands of family photos with pretty good results:
![image](https://user-images.githubusercontent.com/3673134/180773996-e874f801-11c0-4013-9a44-57fe4616049a.png)
![image](https://user-images.githubusercontent.com/3673134/180774049-dfe1f980-1b22-460b-9fb9-288bda9d79c5.png)
![image](https://user-images.githubusercontent.com/3673134/180774153-3136bd1e-cc50-4089-a7e3-2c00bd4deb90.png)
![image](https://user-images.githubusercontent.com/3673134/180774222-e3bdc6f3-53a4-44ae-8c1d-758f3e050d1c.png)

## 3D printing

This is the point at which I acquired a 3D printer (finally!), mostly for other projects but redesigning the scanner in CAD would make everything so much simpler.
The mechanical Lego parts are also slowly turning into a fine dust, and I have one more batch of negatives to scan. Prusa Research happened to run a design contest with cameras as its theme, so I figured it's time to give it a shot!

Some wasted plastic later...

![image](https://user-images.githubusercontent.com/3673134/180776953-c54eb25b-1236-4e41-98d2-0bd05c593c22.png)

And we have a pretty good design, very clean compared to the previous Lego disasters:

![DSCF9565](https://user-images.githubusercontent.com/3673134/180777030-3ded6a04-327b-4c64-b4c7-010ee5940962.JPG)
![image](https://user-images.githubusercontent.com/3673134/180777392-e194bfcd-03da-4d59-888c-e9adbb633422.png)
![image](https://user-images.githubusercontent.com/3673134/180777903-b40e27df-9309-4aa4-a5e8-0d472f858cb0.png)


(yes I still use some lego parts for securing the camera to my table, also duct tape... WIP :-))
