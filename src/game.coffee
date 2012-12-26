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
    @circle = new Circle(
      fillStyle: @colors.red
      radius: 20
      x: 20
      y: 20
    )

    @requestAnimationFrame = @getAnimationRequest()

  colors:
    green: "#33A534"
    black: "#000000"
    red: "#FF2600"

  getAnimationStartTime: -> window.mozAnimationStartTime ||
    window.msAnimationStartTime || window.webkitAnimationStartTime ||
    window.oAnimationStartTime || Date.now()

  start: =>
    @stepStartTime = @getAnimationStartTime()
    setInterval(@gameLoop, 50)

  getUserInput: => @userInput.updateCurrentKeys()

  gameLoop: =>
    @getUserInput()
    @tick()

  tick: => @requestAnimationFrame.call(window, @animate)

  getAnimationRequest: -> window.requestAnimationFrame ||
    window.mozRequestAnimationFrame || window.webkitRequestAnimationFrame ||
    window.msRequestAnimationFrame

  animate: (time) =>
    @clear()
    dyDx = @userInput.getDirection()
    @box.draw(@context, dyDx.dx, dyDx.dy)
    @circle.draw(@context, dyDx.dx, dyDx.dy)

  clear: => @context.clearRect(0, 0, @width, @height)

