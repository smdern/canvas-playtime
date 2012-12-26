class UserInputs
  previousKeys: []
  currentKeys: []

  updateCurrentKeys: =>
    @previousKeys = @currentKeys
    @currentKeys = key.getPressedKeyCodes()

  getDirection: =>
    dy = 0
    dx = 0
    moveFactor = 5
    console.log 'current keys', @currentKeys
    _.each @currentKeys, (key) =>
      switch key
        when 37 # left
          dx -= moveFactor
        when 38 # up
          dy -= moveFactor
        when 39 # right
          dx += moveFactor
        when 40 # down
          dy += moveFactor

    {dx: dx, dy: dy}

