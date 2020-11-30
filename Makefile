compile = xc8-cc -mcpu=16f877a -o dist/lcd

lcd : lcd.p1
	$(compile) dist/lcd.p1

lcd.p1 : src/lcd.c src/lib/lcd.h
	$(compile) -c src/lcd.c

clean :
	rm dist/*
