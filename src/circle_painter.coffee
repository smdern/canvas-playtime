class CirclePainter
  paint: (sprite, context) =>
    x = sprite.left + sprite.width / 2
    y = sprite.top + sprite.height / 2
    radius = sprite.width / 2

    context.save()
    context.beginPath()
    context.arc(x, y, radius, 0, 2 * Math.PI, false)
    context.clip()

    context.shadowColor = 'rgb(0,0,0)'
    context.shadowOffsetX = -4
    context.shadowOffsetY = -4
    context.shadowBlur = 8

    context.fillStyle = Colors.red
    context.fill()

    context.lineWidth = 5
    context.strokeStyle = Colors.black
    context.stroke()

    context.restore()

