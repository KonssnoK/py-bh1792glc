from bh1792glc.driver import *
import logging
import time

def main():
    drv = BH1792GLCDriver()
    
    # Reset the chip just to be sure
    drv.reset()
    time.sleep(0.01)
    
    # Test the chip to be there
    drv.probe()
    
    # Test single measurement to work
    res = drv.measure_single_get()
    logging.error("OFF 0x%X ON 0x%X", res[0], res[1])
    
    # Test synchronized measurement to work
    drv.measure_sync_start()
    
    start_time = time.time()
    while True:
        # Update every 10ms 
        time.sleep(0.010)
        res = drv.measure_sync_ontick()
        
        if(len(res)):
            print(res)
        
        # Do a 30 second acquisition

        cur_time = time.time()
        if((cur_time - start_time) >= 30):
            break
    # Stop the chip
    drv.measure_sync_stop()
    
    # Test unsynchronized measurement to work
    
if __name__ == '__main__':
    main()
