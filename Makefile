CXX = xc8-cc -mcpu=16f877a -o
out = dist

test : test/test.c lib/lcd.h  # dist/lcd.p1
	$(CXX) $(out)/$@ $@/$@.c

# Till after re-write
#
# lcd : lib/lcd.c lib/lcd.h
#	$(CXX) $(out)/$@ -c lib/$@.c

clean :
	rm -v dist/*

