// C3-style driving plot adapted for EON: one mode at a time, sampled and
// rasterized at 10 Hz. Between samples paintEvent only blits the cached image.
void NvgWindow::drawCarrotPlot(QPainter &p) {
  const uint64_t now = millis_since_boot();

  if (!carrot_plot_param_initialized || now - carrot_plot_last_param_read >= 1000) {
    carrot_plot_last_param_read = now;
    carrot_plot_param_initialized = true;
    Params params;
    const std::string value = params.get("ShowPlotMode");
    carrot_plot_mode = std::clamp(value.empty() ? 0 : std::atoi(value.c_str()), 0, 8);
  }

  if (carrot_plot_mode != carrot_plot_mode_prev) {
    carrot_plot_mode_prev = carrot_plot_mode;
    carrot_plot_count = 0;
    carrot_plot_next = 0;
    carrot_plot_last_sample = 0;
    carrot_plot_lead_accel_valid = false;
    carrot_plot_lead_jerk = 0.0f;
    carrot_plot_cache = QImage();
    carrot_plot_cache_size = QSize();
  }
  if (carrot_plot_mode == 0) return;

  const qreal ui_scale = std::min(width() / 1920.0, height() / 1080.0);
  const int graph_left = qRound(360 * ui_scale);
  const int graph_top = qRound(78 * ui_scale);
  // Always reserve the 500 px right navigation/map column, even in ROAD mode.
  const int graph_right_limit = width() - qRound(560 * ui_scale);
  const int graph_width = std::min(qRound(1000 * ui_scale), graph_right_limit - graph_left);
  const int graph_height = qRound(300 * ui_scale);
  if (graph_width < qRound(500 * ui_scale) || graph_height < 120) return;
  const QRect graph_rect(graph_left, graph_top, graph_width, graph_height);

  bool new_sample = false;
  if (carrot_plot_last_sample == 0 || now - carrot_plot_last_sample >= 100) {
    const float sample_dt = carrot_plot_last_sample == 0
        ? 0.1f : std::max(0.05f, (now - carrot_plot_last_sample) * 0.001f);
    carrot_plot_last_sample = now;

    const SubMaster &sm = *(uiState()->sm);
    if (!sm.alive("carState") || !sm.alive("carControl") ||
        !sm.alive("controlsState") || !sm.alive("longitudinalPlan")) {
      return;
    }

    const auto car_state = sm["carState"].getCarState();
    const auto car_control = sm["carControl"].getCarControl();
    const auto controls_state = sm["controlsState"].getControlsState();
    const auto long_plan = sm["longitudinalPlan"].getLongitudinalPlan();
    const auto actuators = car_control.getActuators();
    const auto accels = long_plan.getAccels();
    const auto speeds = long_plan.getSpeeds();
    const float plan_accel = accels.size() > 0 ? accels[0] : 0.0f;
    const float plan_speed = speeds.size() > 0 ? speeds[0] : 0.0f;
    float values[3] = {};

    switch (carrot_plot_mode) {
      case 1: {
        values[0] = car_state.getAEgo();
        values[1] = plan_accel;
        values[2] = actuators.getAccel();
        const QString mpc_mode = long_plan.getMpcMode() > 0 ? "E2E" : "ACC";
        carrot_plot_title = QString("1.Accel [%1 X%2] (Y:actual G:target O:out)")
            .arg(mpc_mode).arg(static_cast<int>(long_plan.getXState()));
        break;
      }
      case 2:
        values[0] = plan_speed;
        values[1] = car_state.getVEgo();
        values[2] = car_state.getAEgo();
        carrot_plot_title = "2.Speed/Accel (Y:target G:vEgo O:aEgo)";
        break;
      case 3: {
        if (!sm.alive("modelV2")) return;
        const auto model = sm["modelV2"].getModelV2();
        const auto position = model.getPosition().getX();
        const auto velocity = model.getVelocity().getX();
        const int p_last = std::max(0, static_cast<int>(position.size()) - 1);
        const int v_last = std::max(0, static_cast<int>(velocity.size()) - 1);
        values[0] = position.size() > 0 ? position[p_last] : 0.0f;
        values[1] = velocity.size() > 0 ? velocity[v_last] : 0.0f;
        values[2] = velocity.size() > 0 ? velocity[0] : 0.0f;
        carrot_plot_title = "3.Model (Y:posEnd G:velEnd O:vel0)";
        break;
      }
      case 4:
      case 5: {
        if (!sm.alive("radarState")) return;
        const auto lead = sm["radarState"].getRadarState().getLeadOne();
        const float lead_accel = lead.getStatus() ? lead.getALeadK() : 0.0f;
        if (lead.getStatus() && carrot_plot_lead_accel_valid) {
          const float raw_jerk = std::clamp(
              (lead_accel - carrot_plot_prev_lead_accel) / sample_dt, -10.0f, 10.0f);
          carrot_plot_lead_jerk = carrot_plot_lead_jerk * 0.8f + raw_jerk * 0.2f;
        } else {
          carrot_plot_lead_jerk = 0.0f;
        }
        carrot_plot_prev_lead_accel = lead_accel;
        carrot_plot_lead_accel_valid = lead.getStatus();
        if (carrot_plot_mode == 4) {
          values[0] = plan_accel;
          values[1] = lead_accel;
          values[2] = lead.getStatus() ? lead.getVRel() : 0.0f;
          carrot_plot_title = "4.Lead (Y:target G:aLead O:vRel)";
        } else {
          values[0] = car_state.getAEgo();
          values[1] = lead_accel;
          values[2] = carrot_plot_lead_jerk;
          carrot_plot_title = "5.Lead jerk (Y:aEgo G:aLead O:jLead)";
        }
        break;
      }
      case 6: {
        const auto torque = controls_state.getLateralControlState().getTorqueState();
        values[0] = torque.getActualLateralAccel() * 10.0f;
        values[1] = torque.getDesiredLateralAccel() * 10.0f;
        values[2] = torque.getOutput() * 10.0f;
        carrot_plot_title = "6.Steer (Y:actual G:desired O:output) x10";
        break;
      }
      case 7: {
        if (!sm.alive("liveParameters")) return;
        const auto live_params = sm["liveParameters"].getLiveParameters();
        values[0] = car_state.getSteeringAngleDeg();
        values[1] = actuators.getSteeringAngleDeg();
        values[2] = live_params.getAngleOffsetDeg() * 10.0f;
        carrot_plot_title = "7.SteerA (Y:actual G:target O:offset*10)";
        break;
      }
      case 8: {
        if (!sm.alive("lateralPlan")) return;
        const auto curvatures = sm["lateralPlan"].getLateralPlan().getCurvatures();
        values[0] = controls_state.getCurvature() * 10000.0f;
        values[1] = controls_state.getDesiredCurvature() * 10000.0f;
        values[2] = (curvatures.size() > 0 ? curvatures[0] : 0.0f) * 10000.0f;
        carrot_plot_title = "8.Curvature (Y:actual G:desired O:plan) x10000";
        break;
      }
      default:
        return;
    }

    for (int channel = 0; channel < 3; ++channel) {
      carrot_plot_samples[channel][carrot_plot_next] = values[channel];
    }
    carrot_plot_next = (carrot_plot_next + 1) % CARROT_PLOT_SAMPLES;
    carrot_plot_count = std::min(carrot_plot_count + 1, CARROT_PLOT_SAMPLES);
    new_sample = true;
  }

  if (new_sample || carrot_plot_cache.isNull() || carrot_plot_cache_size != graph_rect.size()) {
    if (carrot_plot_cache.isNull() || carrot_plot_cache_size != graph_rect.size()) {
      carrot_plot_cache_size = graph_rect.size();
      carrot_plot_cache = QImage(graph_rect.size(), QImage::Format_ARGB32_Premultiplied);
    }
    carrot_plot_cache.fill(Qt::transparent);
    QPainter graph(&carrot_plot_cache);
    graph.setRenderHint(QPainter::Antialiasing);
    graph.setRenderHint(QPainter::TextAntialiasing);
    graph.setPen(QPen(QColor(255, 255, 255, 100), std::max(1, qRound(1 * ui_scale))));
    graph.setBrush(Qt::NoBrush);
    graph.drawRoundedRect(carrot_plot_cache.rect().adjusted(1, 1, -2, -2),
                          qRound(16 * ui_scale), qRound(16 * ui_scale));

    QFont title_font("Open Sans");
    title_font.setPixelSize(std::max(18, qRound(25 * ui_scale)));
    title_font.setBold(true);
    graph.setFont(title_font);
    graph.setPen(Qt::white);
    graph.drawText(QRect(qRound(14 * ui_scale), qRound(5 * ui_scale),
                         graph_rect.width() - qRound(28 * ui_scale), qRound(38 * ui_scale)),
                   Qt::AlignLeft | Qt::AlignVCenter, carrot_plot_title);

    const std::array<QColor, 3> colors = {
      QColor(255, 230, 0), QColor(0, 235, 80), QColor(255, 145, 0)
    };
    const int latest = (carrot_plot_next - 1 + CARROT_PLOT_SAMPLES) % CARROT_PLOT_SAMPLES;
    const int legend_y = qRound(40 * ui_scale);
    int legend_x = qRound(14 * ui_scale);
    QFont value_font("Open Sans");
    value_font.setPixelSize(std::max(17, qRound(23 * ui_scale)));
    value_font.setBold(true);
    graph.setFont(value_font);
    for (int channel = 0; channel < 3; ++channel) {
      const QChar label = channel == 0 ? QChar('Y') : (channel == 1 ? QChar('G') : QChar('O'));
      const QString value = QString("%1:%2").arg(label).arg(
          carrot_plot_count > 0 ? carrot_plot_samples[channel][latest] : 0.0f, 0, 'f', 2);
      graph.setPen(colors[channel]);
      graph.drawText(legend_x, legend_y + QFontMetrics(value_font).ascent(), value);
      legend_x += QFontMetrics(value_font).horizontalAdvance(value) + qRound(20 * ui_scale);
    }

    const QRect plot_rect(qRound(14 * ui_scale), qRound(76 * ui_scale),
                          graph_rect.width() - qRound(28 * ui_scale),
                          graph_rect.height() - qRound(92 * ui_scale));
    float plot_min = -2.0f;
    float plot_max = 2.0f;
    for (int channel = 0; channel < 3; ++channel) {
      for (int i = 0; i < carrot_plot_count; ++i) {
        const int idx = (carrot_plot_next - carrot_plot_count + i + CARROT_PLOT_SAMPLES) % CARROT_PLOT_SAMPLES;
        plot_min = std::min(plot_min, carrot_plot_samples[channel][idx]);
        plot_max = std::max(plot_max, carrot_plot_samples[channel][idx]);
      }
    }
    if (plot_max - plot_min < 1.0f) {
      const float center = (plot_max + plot_min) * 0.5f;
      plot_min = center - 0.5f;
      plot_max = center + 0.5f;
    }

    graph.setPen(QPen(QColor(255, 255, 255, 55), std::max(1, qRound(1 * ui_scale))));
    graph.drawRect(plot_rect);
    if (plot_min < 0.0f && plot_max > 0.0f) {
      const qreal zero_y = plot_rect.bottom() - (-plot_min / (plot_max - plot_min)) * plot_rect.height();
      graph.drawLine(plot_rect.left(), qRound(zero_y), plot_rect.right(), qRound(zero_y));
    }

    for (int channel = 0; channel < 3; ++channel) {
      QPainterPath path;
      for (int i = 0; i < carrot_plot_count; ++i) {
        const int idx = (carrot_plot_next - carrot_plot_count + i + CARROT_PLOT_SAMPLES) % CARROT_PLOT_SAMPLES;
        const qreal x = carrot_plot_count <= 1
            ? plot_rect.left()
            : plot_rect.left() + i * plot_rect.width() / static_cast<qreal>(carrot_plot_count - 1);
        const qreal y = plot_rect.bottom() -
            (carrot_plot_samples[channel][idx] - plot_min) /
            (plot_max - plot_min) * plot_rect.height();
        if (i == 0) path.moveTo(x, y);
        else path.lineTo(x, y);
      }
      graph.setPen(QPen(colors[channel], std::max(2, qRound(3 * ui_scale)),
                        Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      graph.drawPath(path);
    }
    graph.end();
  }

  p.drawImage(graph_rect.topLeft(), carrot_plot_cache);
}
