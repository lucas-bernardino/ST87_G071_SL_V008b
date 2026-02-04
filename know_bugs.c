-KNOWN BUGS

1)  uart_rx_parser_data.rx_ongoing is always TRUE
    //JRF TBF if (uart_tx_handler_data.tx_ongoing || uart_rx_parser_data.rx_ongoing)
    //    return false;