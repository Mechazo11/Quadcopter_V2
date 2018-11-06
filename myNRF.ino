void nrf_setup()
{
radio.begin(); // This must be called first without
// this the radio won't work
radio.openReadingPipe(1,pipe);
radio.startListening();
}
// End of nrf_setup()
void nrf_getcmd()
{
  if (radio.available()) // This function returns true when there is data
  {
    radio.read( data, sizeof(data) ); // Reads the data and puts them in the container
  }
  else //Continuously resets radio. This has drastically improved the auto radio recovery feature of the NRFs
  {
    radio.begin(); // This must be called first without
    // this the radio won't work
    radio.openReadingPipe(1,pipe);
    radio.startListening();
  }
  
  
}

