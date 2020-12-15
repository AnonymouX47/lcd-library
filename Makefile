CXX = xc8-cc -mcpu=16f877a -o
o = dist

test : test/test.c lib/lcd.h  # dist/lcd.p1
	$(CXX) $(o)/$@ $@/$@.c  # $(o)/lcd.p1

# Till after re-write
#
# lcd : lib/lcd.c lib/_lcd.h
#	$(CXX) $(o)/$@ -c lib/$@.c

clean :
	rm dist/*
