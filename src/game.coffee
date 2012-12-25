class Game
  constructor: ->
    $canvas = $('canvas')
    @context = $canvas[0].getContext('2d')
    @width = $canvas.width()
    @height = $canvas.height()
    @userInput = new UserInputs()
    @box = new Box(
      x: 100
      y: 150
      fillStyle: @colors.green
      width: 200
    )

    @requestAnimationFrame = @getAnimationRequest()

  colors:
    green: "#33A534"
    black: "#000000"
    red: "#FF2600"

  getAnimationStartTime: -> window.mozAnimationStartTime ||
    window.msAnimationStartTime || window.webkitAnimationStartTime ||
    window.oAnimationStartTime || Date.now()

  start: => setInterval(@gameLoop, 500)

  getInput: =>
    @userInput.updateCurrentKeys()
    console.log 'previous keys', @userInput.previousKeys
    console.log 'current keys', @userInput.currentKeys

  gameLoop: =>
    @getInput()
    @tick()

  tick: =>
    @stepStartTime = @getAnimationStartTime()
    @requestAnimationFrame.call(window, @animate)

  getAnimationRequest: -> window.requestAnimationFrame ||
    window.mozRequestAnimationFrame || window.webkitRequestAnimationFrame ||
    window.msRequestAnimationFrame

  animate: (timestamp) =>
    dt = timestamp - @stepStartTime
    console.log 'dt', dt
    @clear()
    @box.draw(@context)

  clear: => @context.clearRect(0, 0, @width, @height)

