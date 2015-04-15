# tps23861
Linux driver for TPS23861 IEEE 802.3at Quad Port Power-over-Ethernet PSE Controller.
Tested with kernel version 3.19.0

Bindings example:

tps23861: tps23861@20 {
        compatible = "ti,tps23861";
        reg = <0x20>;
        irq-gpio = <84>;
        port0@0 {
                enable = <1>;
                mode = "auto";
                power = <1>;
        };
        port1@1 {
                enable = <1>;
                mode = "manual";
                power = <0>;
        };
        port2@2 {
                enable = <0>;
        };
        port3@3 {
                enable = <0>;
        };
};
