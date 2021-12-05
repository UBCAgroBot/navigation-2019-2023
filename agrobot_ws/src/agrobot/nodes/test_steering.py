from pynput.keyboard import Key, Listener

def show(key):
    
    pressed_key = str(key).replace("'", "")
    print("\n\n key: ", pressed_key)

    if key == Key.esc:
        # Stop listener
        return False



with Listener(on_press=show) as listener:
    listener.join()
    
