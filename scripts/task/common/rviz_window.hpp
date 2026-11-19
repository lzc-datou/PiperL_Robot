#pragma once
#include <QAction>
#include <QDockWidget>
#include <QGuiApplication>
#include <QMainWindow>
#include <QLayout>
#include <QMenu>
#include <QMenuBar>
#include <QScreen>
#include <QScrollArea>
#include <QTimer>
#include <QVariant>
#include <QWindow>
#include <algorithm>

namespace piper_ui {
inline QScreen *windowScreen(QMainWindow *window, bool primary) {
  if (primary) return QGuiApplication::primaryScreen();
  if (auto *handle = window->windowHandle())
    if (handle->screen()) return handle->screen();
  return QGuiApplication::primaryScreen();
}

// Only add an outer scroll area when an official pane's own minimum would
// prevent fitting the window. Keep its widget instance and signal connections.
inline void allowPlanningDockToShrink(QMainWindow *window, const QSize &space) {
  for (auto *dock : window->findChildren<QDockWidget *>()) {
    if (dock->windowTitle() != "MotionPlanning" && dock->objectName() != "MotionPlanning") continue;
    auto *content = dock->widget();
    if (!content || content->objectName() == "piperPlanningViewport") continue;
    const auto minimum = content->minimumSize().expandedTo(content->minimumSizeHint());
    if (minimum.height() <= space.height() - 160 && minimum.width() <= space.width() * 0.6) continue;
    auto *scroll = new QScrollArea(dock);
    scroll->setObjectName("piperPlanningViewport");
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setMinimumSize(0, 0);
    scroll->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    dock->setWidget(scroll);
    scroll->setWidget(content);
    dock->setMinimumSize(0, 0);
  }
}

inline void cancelStartupFit(QMainWindow *window) {
  if (auto *timer = window->findChild<QTimer *>("piperStartupFitTimer")) {
    timer->stop();
    timer->deleteLater();
  }
}

inline void fitRvizWindow(QMainWindow *window, bool primary) {
  cancelStartupFit(window);
  window->setProperty("piperStartupFitScheduled", true);
  auto *screen = windowScreen(window, primary);
  if (!screen) return;
  // QScreen coordinates are logical pixels. Intersect with this screen because
  // some X11 work-area implementations return a virtual-desktop-sized rectangle.
  auto area = screen->availableGeometry().intersected(screen->geometry()).adjusted(12, 12, -12, -12);
  if (area.isEmpty()) return;
  if (window->isMaximized() || window->isFullScreen()) window->showNormal();
  if (auto *handle = window->windowHandle())
    if (primary && handle->screen() != screen) handle->setScreen(screen);
  allowPlanningDockToShrink(window, area.size());
  window->setMinimumSize(0, 0);
  if (window->layout()) window->layout()->activate();
  const auto margins = window->windowHandle() ? window->windowHandle()->frameMargins() : QMargins();
  const QSize limit(std::max(1, area.width() - margins.left() - margins.right()),
                    std::max(1, area.height() - margins.top() - margins.bottom()));
  window->resize(QSize(1280, 800).boundedTo(limit));
  const auto frame = window->frameGeometry().size();
  window->move(area.x() + std::max(0, (area.width() - frame.width()) / 2),
               area.y() + std::max(0, (area.height() - frame.height()) / 2));
}

inline void installRvizWindowMenu(QMainWindow *window) {
  if (!window) return;
  if (!window->findChild<QMenu *>("piperWindowMenu")) {
    auto *menu = window->menuBar()->addMenu("窗口");
    menu->setObjectName("piperWindowMenu");
    auto *primary = menu->addAction("移到主屏幕");
    auto *current = menu->addAction("适应当前屏幕");
    QObject::connect(primary, &QAction::triggered, window, [window] { fitRvizWindow(window, true); });
    QObject::connect(current, &QAction::triggered, window, [window] { fitRvizWindow(window, false); });
  }
}

inline void prepareRvizWindow(QMainWindow *window, bool fit_on_startup) {
  installRvizWindowMenu(window);
  if (!window) return;
  if (!fit_on_startup) {
    cancelStartupFit(window);
    return;
  }
  if (!window->property("piperStartupFitScheduled").toBool()) {
    window->setProperty("piperStartupFitScheduled", true);
    // RViz restores docks after loading displays. Resize once after that pass;
    // ordinary status refreshes and later dragging never move the window back.
    auto *timer = new QTimer(window);
    timer->setObjectName("piperStartupFitTimer");
    timer->setSingleShot(true);
    QObject::connect(timer, &QTimer::timeout, window, [window] { fitRvizWindow(window, true); });
    timer->start(1200);
  }
}
}  // namespace piper_ui
