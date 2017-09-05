# rys\_sensor\_dwm1000

## TODO
* parameters describing anchors' positions
* poll 3 anchors for distance and return trilaterated position
* improve continuous operation (if needed)

### Building
Download DW1000 API from [DecaWave website](https://www.decawave.com/support/software).

Unpack and copy contents of `decadriver` directory into `./src/deca_api`. It should contain:
* `deca_device_api.h`
* `deca_device.c`
* `deca_params_init.c`
* `deca_param_types.h`
* `deca_regs.h`
* `deca_types.h`
* `deca_version.h`

## License

`DWM.cpp`, `DWM.hpp`, `dwm_platform.c` and `dwm_platform.h` files are licensed under GPLv3 License, as they're based upon [work of Anh Luong and Peter Hillyard](https://github.com/SPAN-UofU/dw1000_bbb).

`DWM.cpp` additionally partially bases upon Decawave's `Simple RX example code` from the API package.

Other files are an original work and are (possibly temporarily) licensed under MIT License.
