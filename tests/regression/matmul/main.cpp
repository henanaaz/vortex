#include <iostream>
#include <unistd.h>
#include <string.h>
#include <vector>
#include <vortex.h>
#include "common.h"
//#define TC_SIZE     2
//#define matrix_size 2

#define RT_CHECK(_expr)                                         \
   do {                                                         \
     int _ret = _expr;                                          \
     if (0 == _ret)                                             \
       break;                                                   \
     printf("Error: '%s' returned %d!\n", #_expr, (int)_ret);   \
	 cleanup();			                                              \
     exit(-1);                                                  \
   } while (false)

///////////////////////////////////////////////////////////////////////////////

const char* kernel_file = "kernel.bin";
uint32_t matrix_size = 0;

vx_device_h device = nullptr;
std::vector<uint8_t> staging_buf;
kernel_arg_t kernel_arg = {};

static void show_usage() {
   std::cout << "Vortex Test." << std::endl;
   std::cout << "Usage: [-k: kernel] [-n words] [-h: help]" << std::endl;
}

static void parse_args(int argc, char **argv) {
  int c;
  while ((c = getopt(argc, argv, "n:k:h?")) != -1) {
    switch (c) {
    case 'n':
      matrix_size = atoi(optarg);
      break;
    case 'k':
      kernel_file = optarg;
      break;
    case 'h':
    case '?': {
      show_usage();
      exit(0);
    } break;
    default:
      show_usage();
      exit(-1);
    }
  }
}

void cleanup() {
  if (device) {
    vx_mem_free(device, kernel_arg.src0_addr);
    vx_mem_free(device, kernel_arg.src1_addr);
    vx_mem_free(device, kernel_arg.dst_addr);
    vx_dev_close(device);
  }
}

//kernel_arg, buf_size, num_points

int run_test(const kernel_arg_t& kernel_arg,
             uint32_t buf_size, 
             uint32_t num_points) {
  // start device
  std::cout << "start device" << std::endl;
  RT_CHECK(vx_start(device));

  // wait for completion
  std::cout << "wait for completion" << std::endl;
  RT_CHECK(vx_ready_wait(device, VX_MAX_TIMEOUT));

  // download destination buffer
  std::cout << "download destination buffer" << std::endl;
  RT_CHECK(vx_copy_from_dev(device, staging_buf.data(), kernel_arg.dst_addr, buf_size));

  // verify result
  std::cout << "verify result" << std::endl;  
  {
    int errors = 0;
    auto buf_ptr = (int32_t*)staging_buf.data();
    uint32_t Ans[TC_SIZE*TC_SIZE] = {108,124,212,244};
    
    for (uint32_t i = 0; i < num_points; ++i) {
      //int ref = i + i; 
      int cur = buf_ptr[i];
      if (cur != Ans[i]) {
        //std::cout << "error at result #" << std::dec << i
        //          << std::hex << ": actual 0x" << cur << ", expected 0x" << ref << std::endl;
        ++errors;
      }
    }
    if (errors != 0) {
      std::cout << "Found " << std::dec << errors << " errors!" << std::endl;
      std::cout << "FAILED!" << std::endl;
      return 1;  
    }
  }

  return 0;
}

int main(int argc, char *argv[]) {  
  // parse command arguments
  parse_args(argc, argv);
  if (matrix_size == 0) {
    matrix_size = 2;
  }

  // open device connection
  std::cout << "open device connection" << std::endl;  
  RT_CHECK(vx_dev_open(&device));

  uint64_t num_cores, num_warps, num_threads;
  RT_CHECK(vx_dev_caps(device, VX_CAPS_NUM_CORES, &num_cores));
  RT_CHECK(vx_dev_caps(device, VX_CAPS_NUM_WARPS, &num_warps));
  RT_CHECK(vx_dev_caps(device, VX_CAPS_NUM_THREADS, &num_threads));

  //Number of tiles * threads
  //uint32_t num_tasks  = num_cores * num_warps * num_threads;
  //TODO - fix this
  uint32_t num_tasks  = ((matrix_size*matrix_size)/(TC_SIZE*TC_SIZE))*num_threads;
  //4*1*1
  uint32_t num_points = TC_SIZE * TC_SIZE;
  //size of each operand
  uint32_t buf_size   = num_points * sizeof(int32_t);

  //64
  std::cout << "number of points: " << num_points << std::endl;
  //256
  std::cout << "buffer size: " << buf_size << " bytes" << std::endl;

  // upload program
  std::cout << "upload program" << std::endl;  
  RT_CHECK(vx_upload_kernel_file(device, kernel_file));

  // allocate device memory
  std::cout << "allocate device memory" << std::endl;
  
  RT_CHECK(vx_mem_alloc(device, buf_size, VX_MEM_TYPE_GLOBAL, &kernel_arg.src0_addr));
  RT_CHECK(vx_mem_alloc(device, buf_size, VX_MEM_TYPE_GLOBAL, &kernel_arg.src1_addr));
  RT_CHECK(vx_mem_alloc(device, buf_size, VX_MEM_TYPE_GLOBAL, &kernel_arg.dst_addr));

  //1
  std::cout << "num_tasks = " << num_tasks << std::endl;
  kernel_arg.num_tasks = num_tasks;
  //1
  kernel_arg.matrix_size = matrix_size;

  std::cout << "dev_src0=0x" << std::hex << kernel_arg.src0_addr << std::endl;
  std::cout << "dev_src1=0x" << std::hex << kernel_arg.src1_addr << std::endl;
  std::cout << "dev_dst=0x" << std::hex << kernel_arg.dst_addr << std::endl;
  std::cout << "num_tasks = " << std::hex << kernel_arg.num_tasks << std::endl;
  std::cout << "matrix_size = " << std::hex << kernel_arg.matrix_size << std::endl;
  
  // allocate staging buffer  
  std::cout << "allocate staging buffer" << std::endl;    
  uint32_t alloc_size = std::max<uint32_t>(buf_size, sizeof(kernel_arg_t));
  staging_buf.resize(alloc_size);
  
  // upload kernel argument
  std::cout << "upload kernel argument" << std::endl;
  memcpy(staging_buf.data(), &kernel_arg, sizeof(kernel_arg_t));
  RT_CHECK(vx_copy_to_dev(device, KERNEL_ARG_DEV_MEM_ADDR, staging_buf.data(), sizeof(kernel_arg_t)));

  // upload source buffer0
  {
    staging_buf.resize(buf_size);
    std::cout << "upload source buffer0" << std::endl;
    auto buf_ptr = (int32_t*)staging_buf.data();
    for (uint32_t i = 0; i < num_points; i+=4) {
      buf_ptr[i+0] = 3;
      buf_ptr[i+1] = 5;
      buf_ptr[i+2] = 7;
      buf_ptr[i+3] = 9;
    }  
    RT_CHECK(vx_copy_to_dev(device, kernel_arg.src0_addr, staging_buf.data(), buf_size));
  }

  // upload source buffer1
  {
    std::cout << "upload source buffer1" << std::endl;
    auto buf_ptr = (int32_t*)staging_buf.data();
    for (uint32_t i = 0; i < num_points; i+=4) {
      buf_ptr[i+0] = 11;
      buf_ptr[i+1] = 13;
      buf_ptr[i+2] = 15;
      buf_ptr[i+3] = 17;
    }  
    RT_CHECK(vx_copy_to_dev(device, kernel_arg.src1_addr, staging_buf.data(), buf_size));
  }

  // clear destination buffer
  //TODO - what is this?
  {
    std::cout << "clear destination buffer" << std::endl;      
    auto buf_ptr = (int32_t*)staging_buf.data();
    for (uint32_t i = 0; i < num_points; ++i) {
      buf_ptr[i] = 0xdeadbeef;
    }  
    RT_CHECK(vx_copy_to_dev(device, kernel_arg.dst_addr, staging_buf.data(), buf_size));  
  }

  // run tests
  std::cout << "run tests" << std::endl;
  RT_CHECK(run_test(kernel_arg, buf_size, num_points));

  // cleanup
  std::cout << "cleanup" << std::endl;  
  cleanup();

  std::cout << "PASSED!" << std::endl;

  return 0;
}