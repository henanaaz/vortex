// Copyright © 2019-2023
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

interface VX_tcu_to_csr_if ();

    wire                                read_enable;
    wire [31:0]                         read_data_a;
    wire [31:0]                         read_data_b;
    wire [`VX_CSR_ADDR_BITS-1:0]        read_addr;

    wire                                write_enable;
    wire [31:0]                         write_data;
    wire [`VX_CSR_ADDR_BITS-1:0]        write_addr;


    //tensor core
    modport master (
        output                          write_enable,
        output                   write_data,   
        output  write_addr,

        output                          read_enable,
        input                    read_data_a,
        input                    read_data_b,
        output read_addr
    );

    //CSR
    modport slave (
        input                           write_enable,
        input                    write_data,   
        input    write_addr,

        input                           read_enable,
        output             read_data_a,
        output               read_data_b,
        input    read_addr
    );

endinterface
