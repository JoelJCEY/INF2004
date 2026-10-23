# PC tests (no Pico needed)

The Code 39 reader is pure logic, so it can be tested on a laptop with
simulated IR_R samples (random letters, both directions, 150-500 mm/s,
speed changes, noise), plus things that must NOT decode (junction
stripes, cut-off barcodes).

```sh
cd tools/host_tests
gcc -O2 -I stub -o test_barcode test_barcode.c ../../app_program/subsystem3/barcode.c
./test_barcode
```

Our result: 2996 ok, 0 wrong, 4 missed of 3000; 0 false reads.
`stub/` holds minimal stand-ins for the kernel headers.
