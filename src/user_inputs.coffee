class UserInputs
  previousKeys: []
  currentKeys: []

  updateCurrentKeys: =>
    @previousKeys = @currentKeys
    @currentKeys = key.getPressedKeyCodes()

  # getKeyChange: => @currentKeys - @previousKeys

