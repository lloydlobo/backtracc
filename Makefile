# Program / executable name.
PROGN := backtracc

# Flags for the build target.
#	-g Compiler flags for debugging.
CFLAGS := -g
SRCS_HEADERS := -lm

build:
	clang -o $(PROGN) $(CFLAGS) main.c $(SRCS_HEADERS)

clean:
	rm -f $(PROGN) *.o

test: CFLAGS += -DVERBOSE_FLAG
test: build
	-(./$(PROGN))


# Additional flag for testing.
#	-DVERBOSE_FLAG for 'test' target build to enable verbose output.
#test: CFLAGS += -DVERBOSE_FLAG
#test: build
#	-(./$(PROGN) tests/input01; \
#		./$(PROGN) tests/input02; \
#		./$(PROGN) tests/input03; \
#		./$(PROGN) tests/input04; \
#		./$(PROGN) tests/input05)
#
#test_output: CFLAGS += -DVERBOSE_FLAG
#test_output: build
#	-(./$(PROGN) tests/output01)
#
