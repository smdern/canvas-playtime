// Generated by CoffeeScript 1.4.0
(function() {
  var circle, clear, draw, init;

  init = function() {
    var $canvas, Context, HEIGHT, WIDTH;
    $canvas = $('canvas');
    Context = _.first($canvas).getContext('2d');
    WIDTH = $canvas.width();
    return HEIGHT = $canvas.height();
  };

  circle = function(x, y, r) {
    Context.beginPath();
    Context.arc(x, y, r, 0, Math.Pi * 2, true);
    Context.closePath();
    return Context.fill();
  };

  clear = function() {
    return Context.clearRect(0, 0, WIDTH, HEIGHT);
  };

  draw = function() {
    clear();
    return circle(5, 5, 10);
  };

  $(document).ready(function() {
    console.log('initializing');
    init();
    console.log('drawing');
    return draw();
  });

}).call(this);
